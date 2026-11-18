import os
import signal
import subprocess
import time
from pathlib import Path

old_pid = 1551444
launch_pid = 1551444
base = Path('/home/nvidia/guide_robot/guide_ws/.codex-backups/nav-20261112-2155')
env = dict(item.split('=', 1) for item in Path(f'/proc/{old_pid}/environ').read_bytes().decode().split('\0') if '=' in item)
env['PYTHONPATH'] = '/home/nvidia/guide_robot/guide_ws/build/guide_voice:' + env.get('PYTHONPATH', '')
cmd = [item for item in Path(f'/proc/{old_pid}/cmdline').read_bytes().decode().split('\0') if item]
param_index = cmd.index('--params-file') + 1
params = base / 'previous-params.yaml'
params.write_bytes(Path(cmd[param_index]).read_bytes())
cmd[param_index] = str(params)
cmd += ['-p', 'startup_goto_standby:=false', '-p', 'auto_relocalize:=false']
cwd = os.readlink(f'/proc/{old_pid}/cwd')
os.kill(launch_pid, signal.SIGINT)
for _ in range(20):
    if not Path(f'/proc/{old_pid}').exists():
        break
    time.sleep(.5)
else:
    raise SystemExit('Old node still exists; did not start a duplicate.')
log = open(base / 'restarted-voice.log', 'ab', buffering=0)
proc = subprocess.Popen(cmd, cwd=cwd, env=env, stdin=subprocess.DEVNULL,
                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
(base / 'restarted-voice.pid').write_text(str(proc.pid)+'\n')
print('Started voice node PID', proc.pid, 'with startup movement and startup relocalization disabled.')
time.sleep(3)
print('running:', proc.poll() is None)
