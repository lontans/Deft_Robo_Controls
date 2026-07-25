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
cmd = """
ps -ef | grep -E 'pdb_uart_sim|debug_dashboard' | grep -v grep || echo 'no matching procs'
echo ---
tail -30 /tmp/pdb_uart_sim.log 2>/dev/null || echo no_log
echo ---
ss -ltn | grep -E '8765|8766|8767' || echo 'no 876x listeners'
echo ---
cd ~/controls_pcb && python3 scripts/soft_dfu_flash.py scan
"""
_, o, e = c.exec_command(cmd, timeout=30)
print(o.read().decode(errors="replace"))
err = e.read().decode(errors="replace")
if err.strip():
    print("ERR", err[:500])
c.close()
