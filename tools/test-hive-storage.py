#!/usr/bin/env python3
"""Storage report and prune against a fake adb whose "Hub" is a folder on this computer.
The fake answers the exact commands the tool sends; it does not prove how the real Hub's
shell behaves (that needs one run on the robot)."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent
TOOL = ROOT / 'tools' / 'hive-storage.py'
FAKE = r'''#!/usr/bin/env python3
import hashlib, os, re, sys
from pathlib import Path
hub = Path(os.environ['TEST_HUB'])            # stands in for /sdcard/FIRST/pollen-hive
a = sys.argv[1:]
assert a[0] == '-s' and a[1] == 'fake-hub:5555' and a[2] == 'shell', a
command = a[3]
with open(os.environ['TEST_ADB_LOG'], 'a') as f: f.write(command + '\n')
if command.startswith('df -k'):
    print('Filesystem     1K-blocks    Used Available Use% Mounted on')
    print('/data/media      5242880 1048576 %s  20%% /storage/emulated' % os.environ.get('TEST_FREE_KB', '4194304'))
elif command.startswith('du -k /sdcard/FIRST/pollen-hive'):
    for folder in sorted(p for p in hub.iterdir() if p.is_dir()):
        kb = sum(f.stat().st_size for f in folder.iterdir()) // 1024
        print('%d\t/sdcard/FIRST/pollen-hive/%s' % (kb, folder.name))
    print('9\t/sdcard/FIRST/pollen-hive')
elif command.startswith('du -k /sdcard/robotControllerLog'):
    print('4096\t/sdcard/robotControllerLog.txt')
elif 'md5sum' in command:
    if os.environ.get('TEST_NO_MD5'): sys.exit(127)
    for folder in re.search(r'find (.*?) -type f', command).group(1).split():
        for f in sorted((hub / folder).glob('*')) if (hub / folder).is_dir() else []:
            print('%s  %s/%s' % (hashlib.md5(f.read_bytes()).hexdigest(), folder, f.name))
elif command.startswith('rm -f '):
    for path in re.findall(r"'([^']+)'", command):
        assert path.startswith('/sdcard/FIRST/pollen-hive/'), path
        (hub / path[len('/sdcard/FIRST/pollen-hive/'):]).unlink()
else:
    sys.exit(2)
'''


def run(*arguments, **extra):
    return subprocess.run([sys.executable, str(TOOL), *arguments], env=dict(env, **extra), capture_output=True, text=True)


with tempfile.TemporaryDirectory() as tmp:
    folder = Path(tmp)
    adb = folder / 'adb'
    adb.write_text(FAKE)
    adb.chmod(0o755)
    hub, session = folder / 'hub', folder / 'session'
    files = {'teleop/teleop-20261116-100000.csv': b't' * 300000, 'auto/auto-20261116-100100.csv': b'a' * 5000,
             'traces/match-1.txt.csv': b'x' * 9000, 'reports/match-1.txt': b'r' * 400,
             'benchmark/hive-1.jsonl': b'b' * 70000}
    for name, data in files.items():
        for base in (hub, session / 'pollen-hive'):
            (base / name).parent.mkdir(parents=True, exist_ok=True)
            (base / name).write_bytes(data)
    (hub / 'camera.properties').write_text('pipeline=4\n')
    # On the Hub but not safely downloaded: a new run, and one that grew after the download.
    (hub / 'teleop/teleop-20261116-110000.csv').write_bytes(b'new run')
    (hub / 'traces/match-1.txt.csv').write_bytes(b'x' * 9000 + b'more rows')
    (hub / 'teleop/odd name.csv').write_bytes(b'?')
    log = folder / 'adb.log'
    env = dict(os.environ, FTC_ADB=str(adb), FTC_DEVICE='fake-hub:5555', TEST_HUB=str(hub), TEST_ADB_LOG=str(log))

    result = run('report')
    assert result.returncode == 0 and 'Free on /sdcard: 4.00 GB' in result.stdout, result.stdout + result.stderr
    assert 'teleop' in result.stdout and 'of 128 MB' in result.stdout and 'OK: no folder is near its budget' in result.stdout
    low = run('report', TEST_FREE_KB='204800')
    assert 'WARNING: less than 500 MB free' in low.stdout, low.stdout
    assert 'rm ' not in log.read_text(), 'report is read-only'

    # Dry run: lists, removes nothing.
    before = sorted(p.relative_to(hub) for p in hub.rglob('*') if p.is_file())
    result = run('prune', str(session))
    assert result.returncode == 0 and 'Nothing was removed' in result.stdout, result.stdout + result.stderr
    assert 'would remove teleop/teleop-20261116-100000.csv' in result.stdout
    assert 'keep   teleop/teleop-20261116-110000.csv: not in the download' in result.stdout
    assert 'keep   traces/match-1.txt.csv: differs from the download' in result.stdout
    assert before == sorted(p.relative_to(hub) for p in hub.rglob('*') if p.is_file()) and 'rm ' not in log.read_text()

    # If the Hub cannot checksum, nothing is removed even with --delete.
    result = run('prune', str(session), '--delete', TEST_NO_MD5='1')
    assert 'Nothing was removed' in result.stdout and 'rm ' not in log.read_text(), result.stdout
    # A folder that is not a download is refused.
    result = run('prune', str(folder / 'hub' / 'reports'), '--delete')
    assert result.returncode == 1 and 'rm ' not in log.read_text(), result.stdout + result.stderr

    # Real prune: only the four identical files go; settings, new and changed files stay.
    result = run('prune', str(session / 'pollen-hive'), '--delete')
    assert result.returncode == 0 and 'Removed 4 recordings' in result.stdout, result.stdout + result.stderr
    left = sorted(str(p.relative_to(hub)) for p in hub.rglob('*') if p.is_file())
    assert left == ['camera.properties', 'teleop/odd name.csv', 'teleop/teleop-20261116-110000.csv',
                    'traces/match-1.txt.csv'], left
    assert all((session / 'pollen-hive' / name).is_file() for name in files), 'the download is untouched'
    assert 'Control Hub storage' in result.stdout, 'prune ends with the new storage report'
    # Without the ftc.sh environment the tool explains itself instead of guessing a device.
    bare = subprocess.run([sys.executable, str(TOOL), 'report'], capture_output=True, text=True,
                          env={k: v for k, v in os.environ.items() if not k.startswith('FTC_')})
    assert bare.returncode != 0 and 'tools/ftc.sh' in bare.stderr
print('PASS hive storage: read-only report, dry run, checksum-verified prune, settings and unverified files kept')
