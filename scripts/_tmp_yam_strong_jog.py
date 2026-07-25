
import sys, time
from pathlib import Path
sys.path.insert(0, str(Path('.').resolve()))
import numpy as np
from deft_controls_sdk import ActuatorDesire, McuState
from deft_controls_sdk.bench.soft_dfu import find_cdc_port
from deft_controls_sdk.link.exchange import ACTUATOR_COUNT
from deft_controls_sdk.vbeta import PcbArmDriver, PcbRobotSession, ensure_yam_left_arm_cfg
from deft_controls_sdk.vbeta import slots as yam_slots
from deft_controls_sdk.vbeta.cfg import pause_plant_stream

STREAM_HZ = 12.0
KP = (80.0, 55.0, 100.0, 120.0, 100.0, 100.0, 160.0)
KD = 4.0

def cfg_en(hub, en):
    for i in range(7):
        hub.debug.cfg_set_slot(slot=i,bus=1,protocol=yam_slots.PROTO_DAMIAO,motor_id=0x01+i,master_id=yam_slots._DAMIAO_MASTER[i],enabled=(i in en),persist=False)

def faults(arm):
    fb=arm._session.latest_feedback();
    return [int(fb.actuator(s).fault) if fb and fb.actuator(s) else -1 for s in arm.slots]

port=find_cdc_port()
with PcbRobotSession.connect(port, apply_yam_cfg=False, stream_hz=STREAM_HZ, idle_first=True) as session:
    hub=session.hub
    with pause_plant_stream(hub):
        ensure_yam_left_arm_cfg(hub, force=True)
        cfg_en(hub, set())
    arm=PcbArmDriver(session, side='left', skip_home_on_connect=True, clamp_goals=False, kp=KP, kd=KD)
    arm.is_connected=True
    armed=set(); qh=np.zeros(7)
    for i in range(7):
        armed.add(i)
        with pause_plant_stream(hub): cfg_en(hub, armed)
        hub.set_mcu_state(McuState.NORMAL, send=True)
        fb=session.latest_feedback()
        for s in armed:
            st=fb.actuator(s) if fb else None
            if st: qh[s]=float(st.position)
        for _ in range(30):
            arm.write('Goal_Position', qh.astype(np.float32)); time.sleep(0.08)
            if all(faults(arm)[s]==1 for s in armed): break
        print('arm',sorted(armed),'f',faults(arm))
    q0=np.asarray(arm.read('Position_Rad'),dtype=np.float64)
    print('q0',q0)
    # solo stronger jogs on J2/J4/J7 while keeping all CFG on
    for j,d in [(1,0.15),(3,0.15),(6,0.25)]:
        for sign in (+1,-1):
            q=q0.copy(); q[j]=q0[j]+sign*d
            print(f'J{j+1} cmd {q[j]:+.4f} from {q0[j]:+.4f}')
            t0=time.perf_counter()
            while time.perf_counter()-t0 < 2.5:
                arm.write('Goal_Position', q.astype(np.float32)); time.sleep(0.08)
            q1=np.asarray(arm.read('Position_Rad'),dtype=np.float64)
            tau=np.asarray(arm.read('torque'),dtype=np.float64)
            held=session.hub.held_desire(j)
            print(f'  dq={q1[j]-q0[j]:+.4f} tau={tau[j]:+.3f} held_p={held.position if held else None} f={faults(arm)}')
            t0=time.perf_counter()
            while time.perf_counter()-t0 < 1.2:
                arm.write('Goal_Position', q0.astype(np.float32)); time.sleep(0.08)
    hub.set_mcu_state(McuState.DIAG_ONLY, send=True)
    for s in range(ACTUATOR_COUNT): hub.set_actuator(s, ActuatorDesire(), send=False)
    session.send_once(); time.sleep(0.1); hub.recover()
print('done')
