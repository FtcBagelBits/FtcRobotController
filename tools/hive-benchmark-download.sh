#!/usr/bin/env bash
# Run after STOP; explicit device selection avoids accidentally pulling an emulator.
set -euo pipefail
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo 'Usage: bash tools/hive-benchmark-download.sh NEW_OUTPUT_FOLDER [ADB_DEVICE]' >&2
  exit 2
fi
out=$1
device=${2:-192.168.43.1:5555}
if [ -e "$out" ] || [ -e "$out.zip" ]; then echo 'Choose a new output folder (no overwrites).' >&2; exit 1; fi
command -v adb >/dev/null
command -v zip >/dev/null
case "$device" in *:*) adb connect "$device" ;; esac
adb -s "$device" get-state
mkdir -p "$out"
adb -s "$device" pull /sdcard/FIRST/pollen-hive "$out/pollen-hive"
# Current logcat is optional; benchmark files and saved camera settings are mandatory.
adb -s "$device" logcat -d -v threadtime > "$out/logcat.txt" 2> "$out/logcat-error.txt" || true
{
  git rev-parse HEAD
  git status --short
} > "$out/checkout.txt"
if ! python3 "$(dirname "$0")/hive-benchmark-analyze.py" "$out/pollen-hive" --out "$out/analysis" > "$out/analysis-output.txt" 2>&1; then
  echo "Analysis could not finish; raw logs are preserved. See analysis-output.txt." >&2
fi
(cd "$(dirname "$out")" && zip -qr "$(basename "$out").zip" "$(basename "$out")")
printf 'Download complete: %s.zip\nAlso upload the pipeline export folder.\n' "$out"
