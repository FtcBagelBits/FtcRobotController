#!/usr/bin/env python3
"""Make the hive settings saved on the Control Hub the new code defaults.

The hive autos and TEST Hive camera save only the rows that differ from their defaults, in
FIRST/pollen-hive on the Hub: camera.properties (all hive modes), primary.properties,
secondary.properties. This rewrites the matching row(...) defaults in PollenHiveSettings.java.

Usage (from the repo):
    adb pull /sdcard/FIRST/pollen-hive ~/Desktop/pollen-hive
    python3 tools/hive-promote-defaults.py ~/Desktop/pollen-hive            # show what would change
    python3 tools/hive-promote-defaults.py ~/Desktop/pollen-hive --write    # change the Java file
Then: bash tools/test-pollen-sequence.sh, commit, install. The promoted rows stop showing CHANGED.
"""
import argparse
import math
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
JAVA = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenHiveSettings.java'
NOT_SAVED = {'execution'}
NUM = r'(Double\.NaN|-?[0-9.]+(?:[eE]-?[0-9]+)?)'


def read_properties(path):
    """The Hub writes plain key=value lines with simple keys; # lines are comments."""
    values = {}
    if not path.is_file():
        return values
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line[0] in '#!' or '=' not in line:
            continue
        key, value = line.split('=', 1)
        values[key.strip()] = float(value.strip())
    return values


def java_number(x):
    if math.isnan(x):
        return 'Double.NaN'
    text = repr(round(x, 10))
    return text[:-2] if text.endswith('.0') and abs(x) >= 1 else text


def parse_java(src):
    rows = {}
    for m in re.finditer(r'row\("(\w+)",\s*"[^"]*",\s*' + NUM + r',\s*' + NUM + ',', src):
        rows[m.group(1)] = (float('nan') if 'NaN' in m.group(2) else float(m.group(2)),
                            float('nan') if 'NaN' in m.group(3) else float(m.group(3)), m)
    return rows


def same(a, b):
    return math.isnan(a) and math.isnan(b) or (not math.isnan(a) and not math.isnan(b) and abs(a - b) < 1e-9)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', type=pathlib.Path, help='the pulled pollen-hive folder')
    ap.add_argument('--write', action='store_true', help='change PollenHiveSettings.java (default: only show)')
    ap.add_argument('--java', type=pathlib.Path, default=JAVA, help=argparse.SUPPRESS)
    args = ap.parse_args()
    src = args.java.read_text()
    rows = parse_java(src)
    # file -> which default column(s) it sets: camera sets both, primary the first, secondary the second
    plan = []
    for name, columns in (('camera', (0, 1)), ('primary', (0,)), ('secondary', (1,))):
        for key, value in read_properties(args.folder / (name + '.properties')).items():
            if key in NOT_SAVED:
                continue
            if key not in rows:
                print(f'SKIP {name}.{key}: no such setting in this code')
                continue
            for col in columns:
                if not same(rows[key][col], value):
                    plan.append((key, col, rows[key][col], value, name))
    if not plan:
        print('Nothing to promote: the saved changes already match the code defaults.')
        return 0
    new = {}
    for key, col, old, value, name in plan:
        new.setdefault(key, [rows[key][0], rows[key][1]])[col] = value
        print(f'{key:16s} {"primary" if col == 0 else "secondary":9s} {java_number(old):>14s} -> {java_number(value):<14s} (from {name})')
        if key == 'side':
            print('  WARNING: a RED or BLUE side default forces that side in EVERY match. Usually leave it AUTO:'
                  ' reset the Side row on the robot (Triangle, Square) and pull again.')
    if not args.write:
        print('\nShown only. Add --write to change PollenHiveSettings.java.')
        return 0
    for key, (p, s) in new.items():
        pattern = re.compile(r'(row\("' + key + r'",\s*"[^"]*",\s*)' + NUM + r'(,\s*)' + NUM + ',')
        src, n = pattern.subn(lambda m: m.group(1) + java_number(p) + m.group(3) + java_number(s) + ',', src, count=1)
        if n != 1:
            sys.exit(f'Could not rewrite {key}')
    args.java.write_text(src)
    print(f'\nUpdated {args.java.relative_to(ROOT) if args.java.is_relative_to(ROOT) else args.java}. '
          'Next: bash tools/test-pollen-sequence.sh, commit, install.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
