import os
import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(
    "192.168.50.48",
    username="deft-robotics",
    password=os.environ.get("JETSON_PASS", "4565"),
    timeout=15,
)
for cmd in [
    "ps -ef | grep -E 'debug_dashboard|yam_continuous|vbeta|deft_vbeta|soft_dfu|pdb_uart' | grep -v grep || true",
    "fuser /dev/ttyACM* 2>/dev/null || true",
    "cd ~/controls_pcb && python3 scripts/soft_dfu_flash.py scan",
    "ls -lt ~/controls_pcb/scripts/deft_controls_sdk/debug_dashboard/ | head -20",
    "test -f ~/controls_pcb/.deft_session/state.json && python3 -c \"import json; d=json.load(open('/home/deft-robotics/controls_pcb/.deft_session/state.json')); print({k:d.get(k) for k in ['mcu_state','kill_state','port','connected','fault_count'] if k in d or True}); print('keys', sorted(d)[:40])\" || echo no_state",
]:
    print(">>>", cmd)
    _, o, e = c.exec_command(cmd, timeout=30)
    print(o.read().decode(errors="replace"))
    err = e.read().decode(errors="replace")
    if err.strip():
        print("ERR", err[:500])
c.close()
