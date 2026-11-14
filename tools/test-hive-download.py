#!/usr/bin/env python3
"""Exercise downloader success, optional failures, offline state and no-overwrite with fake adb."""
import os
from pathlib import Path
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parent.parent
FAKE = '''#!/usr/bin/env python3
import os, sys
from pathlib import Path
a=sys.argv[1:]
if a[0]=='connect': print('connected'); sys.exit(0)
if a[0]=='-s': a=a[2:]
if a[0]=='get-state':
 print('offline' if os.environ.get('TEST_OFFLINE') else 'device'); sys.exit(0)
if a[:2]==['pull','/sdcard/FIRST/pollen-hive']:
 p=Path(a[2]); (p/'traces').mkdir(parents=True); (p/'reports').mkdir()
 (p/'camera.properties').write_text('pipeline=4\\n')
 (p/'reports'/'match.txt').write_text('START build=pollen-hive-v7\\n')
 (p/'traces'/'match.txt.csv').write_text('t_s,phase,kind,reason,decision_inputs,decision_launcher_tps\\n1,COAST,SAMPLE,waiting,evaluated=COAST coast_speed_ok=false,101\\n2,PARK,TRANSITION,flywheel did not stop,evaluated=COAST coast_speed_ok=false,101\\n')
 sys.exit(0)
if a[0]=='shell' and a[1]=='ls /sdcard': print('robotControllerLog.txt'); sys.exit(0)
if a[0]=='shell' and a[1]=='ls /sdcard/FIRST': print('robot.xml'); sys.exit(0)
if a[:2]==['pull','/sdcard/robotControllerLog.txt'] or a[:2]==['pull','/sdcard/FIRST/robot.xml']:
 (Path(a[2])/Path(a[1]).name).write_text('fixture'); sys.exit(0)
if a[0]=='shell': print('fixture status'); sys.exit(0)
if a[0]=='logcat': print('fixture logcat'); sys.exit(0)
sys.exit(1)
'''

with tempfile.TemporaryDirectory() as tmp:
    folder = Path(tmp)
    adb = folder / 'adb'
    adb.write_text(FAKE)
    adb.chmod(0o755)
    env = dict(os.environ, PATH=str(folder) + os.pathsep + os.environ['PATH'])
    output = folder / 'session'
    cmd = ['bash', str(ROOT / 'tools/hive-benchmark-download.sh'), str(output)]
    result = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr + result.stdout
    with zipfile.ZipFile(str(output) + '.zip') as z:
        for name in ['pollen-hive/traces/match.txt.csv', 'pollen-hive/camera.properties',
                     'robot-logs/robotControllerLog.txt', 'configs/robot.xml', 'storage.txt', 'logcat.txt']:
            assert 'session/' + name in z.namelist(), name
    assert '101.0' in (output / 'auto-summary.md').read_text()
    assert 'flywheel did not stop' in (output / 'auto-summary.md').read_text()
    assert subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True).returncode != 0
    offline = folder / 'offline'
    result = subprocess.run(cmd[:-1] + [str(offline)], cwd=ROOT,
                            env=dict(env, TEST_OFFLINE='1'), capture_output=True, text=True)
    assert result.returncode != 0 and 'not online' in result.stderr
    assert not offline.exists()
print('PASS hive download: raw files, optional failures, summary, no-overwrite and offline handling')
