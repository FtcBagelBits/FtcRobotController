#!/usr/bin/env python3
"""Control Hub storage: how full is it, and free space by removing recordings that are safely
on the laptop.

  python3 tools/hive-storage.py report
      Read-only. Free space on the Hub and the size of every recording folder against its budget.

  python3 tools/hive-storage.py prune DOWNLOADED_FOLDER [--delete]
      Compares every recording on the Hub with the copy in DOWNLOADED_FOLDER (a folder made by
      tools/ftc.sh logs). Without --delete it only lists what it would remove. With --delete it
      removes from the Hub ONLY files whose MD5 checksum matches the downloaded copy.
      It never touches the saved settings (*.properties), robot configurations or SDK logs.

Run after the OpMode is STOPPED: a recording that is still being written must not be removed.
Normally run through tools/ftc.sh (storage / prune), which finds adb and connects to the Hub.
The adb program and device come from FTC_ADB and FTC_DEVICE.
"""
import argparse
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys

HIVE = '/sdcard/FIRST/pollen-hive'
MIB = 1024 * 1024
# Folder -> budget in MiB (0 = no folder budget). Keep in step with PollenRunLog.Kind.
FOLDERS = {'teleop': 128, 'auto': 32, 'traces': 64, 'reports': 0, 'benchmark': 0}
SAFE_NAME = re.compile(r'^[A-Za-z0-9._-]+$')


def adb(*arguments, timeout=900):
    program, device = os.environ.get('FTC_ADB'), os.environ.get('FTC_DEVICE')
    if not program or not device:
        sys.exit('Run this through tools/ftc.sh (storage or prune): FTC_ADB and FTC_DEVICE are not set.')
    return subprocess.run([program, '-s', device, *arguments], capture_output=True, text=True, timeout=timeout)


def shell(command, timeout=900):
    return adb('shell', command, timeout=timeout).stdout.replace('\r', '')


def free_kb():
    """Free kilobytes on /sdcard, or None if df's answer cannot be read."""
    lines = [line.split() for line in shell('df -k /sdcard').splitlines() if line.strip()]
    if len(lines) < 2:
        return None, ''
    header, values = lines[0], lines[-1]
    raw = ' '.join(values)
    for name in ('Available', 'Avail', 'Free'):
        if name in header and header.index(name) < len(values) and values[header.index(name)].isdigit():
            return int(values[header.index(name)]), raw
    return None, raw


def folder_kb():
    """{folder name: kilobytes} for the recording folders that exist on the Hub."""
    sizes = {}
    for line in shell(f'du -k {HIVE}').splitlines():
        parts = line.split(None, 1)
        if len(parts) == 2 and parts[0].isdigit() and parts[1].startswith(HIVE + '/'):
            name = parts[1][len(HIVE) + 1:]
            if name in FOLDERS:
                sizes[name] = int(parts[0])
    return sizes


def report():
    free, raw = free_kb()
    print('Control Hub storage (read-only check)')
    print(f'  Free on /sdcard: {free / 1048576:.2f} GB' if free is not None else f'  Free on /sdcard: could not read df ({raw})')
    sizes = folder_kb()
    if not sizes:
        print(f'  No recording folders under {HIVE} yet (or the Hub did not answer du).')
    warnings = []
    for name, budget in FOLDERS.items():
        kb = sizes.get(name, 0)
        line = f'  {name:<10}{kb / 1024:8.1f} MB'
        if budget:
            share = kb / 1024 / budget
            line += f' of {budget} MB ({share:.0%})'
            if share >= 0.8:
                warnings.append(f'{name} is {share:.0%} of its budget')
        print(line)
    logs = shell('du -k /sdcard/robotControllerLog.txt* /sdcard/FIRST/matchlogs 2>/dev/null')
    sdk_kb = sum(int(line.split()[0]) for line in logs.splitlines() if line.split() and line.split()[0].isdigit())
    print(f'  SDK logs  {sdk_kb / 1024:8.1f} MB (robotControllerLog.txt and matchlogs; the SDK limits these itself)')
    if free is not None and free < 500 * 1024:
        warnings.append('less than 500 MB free')
    if warnings:
        print('WARNING: ' + '; '.join(warnings) + '. Download (tools/ftc.sh logs), then tools/ftc.sh prune.')
    else:
        print('OK: no folder is near its budget.')
    return 0


def hub_checksums():
    """{relative path: md5} for every recording file on the Hub."""
    folders = ' '.join(FOLDERS)
    listing = shell(f'cd {HIVE} && find {folders} -type f -exec md5sum {{}} \\; 2>/dev/null')
    found = {}
    for line in listing.splitlines():
        parts = line.split(None, 1)
        if len(parts) == 2 and re.fullmatch(r'[0-9a-f]{32}', parts[0]):
            found[parts[1].strip().lstrip('*')] = parts[0]
    return found


def local_md5(path):
    digest = hashlib.md5()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def prune(downloaded, delete):
    local = downloaded / 'pollen-hive' if (downloaded / 'pollen-hive').is_dir() else downloaded
    if not any((local / name).is_dir() for name in FOLDERS):
        print(f'{downloaded} is not a downloaded session: no recording folders in it. Nothing was removed.', file=sys.stderr)
        return 1
    on_hub = hub_checksums()
    if not on_hub:
        print('No recordings found on the Hub (or the Hub could not checksum them). Nothing was removed.')
        return 0
    verified, kept = [], []
    for relative, checksum in sorted(on_hub.items()):
        folder, _, name = relative.partition('/')
        copy = local / folder / name
        if folder not in FOLDERS or not SAFE_NAME.match(name):
            kept.append((relative, 'unexpected name'))
        elif not copy.is_file():
            kept.append((relative, 'not in the download'))
        elif local_md5(copy) != checksum:
            kept.append((relative, 'differs from the download (newer or still being written)'))
        else:
            verified.append((relative, copy.stat().st_size))
    total = sum(size for _, size in verified)
    print(f'{len(verified)} recordings on the Hub match the download ({total / MIB:.1f} MB); {len(kept)} do not and stay.')
    for relative, why in kept:
        print(f'  keep   {relative}: {why}')
    if not delete:
        for relative, size in verified:
            print(f'  would remove {relative} ({size / 1024:.0f} kB)')
        print('Nothing was removed. Add --delete to remove the matching files from the Hub.')
        return 0
    if not verified:
        return 0
    paths = [f"'{HIVE}/{relative}'" for relative, _ in verified]
    for start in range(0, len(paths), 20):
        adb('shell', 'rm -f ' + ' '.join(paths[start:start + 20]))
    left = set(hub_checksums()) & {relative for relative, _ in verified}
    print(f'Removed {len(verified) - len(left)} recordings from the Hub. The copies in {local} are now the only ones: keep them.')
    if left:
        print(f'{len(left)} could not be removed: ' + ', '.join(sorted(left)), file=sys.stderr)
        return 1
    return report()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('report')
    pruning = commands.add_parser('prune')
    pruning.add_argument('downloaded', type=Path)
    pruning.add_argument('--delete', action='store_true')
    args = parser.parse_args()
    try:
        sys.exit(report() if args.command == 'report' else prune(args.downloaded, args.delete))
    except subprocess.TimeoutExpired:
        sys.exit('The Hub did not answer in time. Nothing further was done.')


if __name__ == '__main__':
    main()
