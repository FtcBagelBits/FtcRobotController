#!/usr/bin/env python3
"""Read-only Limelight export: all saved pipelines, status, hardware report and calibration.
Connect the Limelight to the Mac using its normal data connection before running.
Does not alter camera settings. HTTP API port 5807. No third-party modules.
"""
import argparse
import json
from pathlib import Path
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='limelight.local', help='Camera hostname/IP, not the Control Hub address')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if not args.host or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-' for c in args.host):
        parser.error('Use a hostname or IPv4 address without scheme, port or path')
    args.out.mkdir(parents=True, exist_ok=False)
    errors = {}
    routes = {f'pipeline-{i}.json': f'/pipeline-atindex?index={i}' for i in range(10)}
    routes.update({'status.json': '/status', 'hwreport.json': '/hwreport', 'cal-file.json': '/cal-file', 'cal-eeprom.json': '/cal-eeprom'})
    for filename, route in routes.items():
        try:
            with urllib.request.urlopen(f'http://{args.host}:5807{route}', timeout=3) as response:
                data = json.load(response)
            (args.out/filename).write_text(json.dumps(data, indent=2)+'\n')
            print('Saved', filename)
        except Exception as exc:
            errors[filename] = str(exc)
            print('UNAVAILABLE', filename, exc)
    (args.out/'export-info.json').write_text(json.dumps({'host': args.host, 'errors': errors}, indent=2)+'\n')
    if not any((args.out/f'pipeline-{i}.json').exists() for i in range(10)):
        parser.exit(1, 'No pipeline exports. Use the web UI download button as fallback; preserve export-info.json.\n')
    print('Saved settings are from disk; commit UI changes before exporting. Keep this folder with the run logs.')


if __name__ == '__main__':
    main()
