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
# A shell alias is not inherited by bash scripts; use the SDK binary when needed.
if ! command -v adb >/dev/null 2>&1; then
  if [ -x "$HOME/Library/Android/sdk/platform-tools/adb" ]; then
    export PATH="$HOME/Library/Android/sdk/platform-tools:$PATH"
  else
    echo 'adb not found. Add Android SDK platform-tools to PATH.' >&2; exit 1
  fi
fi
printf 'Connecting to %s...\n' "$device"
command -v zip >/dev/null
case "$device" in *:*) adb connect "$device" ;; esac
if [ "$(adb -s "$device" get-state 2>/dev/null | tr -d '\r')" != device ]; then
  echo 'Hub is not online. Rejoin Hub Wi-Fi; run adb disconnect then adb connect; check adb devices -l.' >&2
  exit 1
fi
mkdir -p "$out"
echo "Downloading saved settings, reports, benchmarks and auto traces..."
adb -s "$device" pull /sdcard/FIRST/pollen-hive "$out/pollen-hive"
echo 'Downloading SDK logs and storage status...'
adb -s "$device" shell 'df -h /sdcard; du -k /sdcard/FIRST/pollen-hive' > "$out/storage.txt" 2>&1 || true
mkdir -p "$out/robot-logs" "$out/configs"
# SDK rotations and hardware XML help diagnose resets, configuration and motor mapping.
# Optional pulls never discard the mandatory hive folder if a file is absent.
while IFS= read -r name; do
  case "$name" in robotControllerLog*)
    adb -s "$device" pull "/sdcard/$name" "$out/robot-logs/" >> "$out/optional-pulls.txt" 2>&1 || true ;;
  esac
done < <(adb -s "$device" shell 'ls /sdcard' 2>/dev/null | tr -d '\r')
while IFS= read -r name; do
  case "$name" in *.xml)
    adb -s "$device" pull "/sdcard/FIRST/$name" "$out/configs/" >> "$out/optional-pulls.txt" 2>&1 || true ;;
  esac
done < <(adb -s "$device" shell 'ls /sdcard/FIRST' 2>/dev/null | tr -d '\r')
adb -s "$device" pull /sdcard/FIRST/matchlogs "$out/robot-logs/matchlogs" >> "$out/optional-pulls.txt" 2>&1 || true
adb -s "$device" shell dumpsys package com.qualcomm.ftcrobotcontroller > "$out/installed-app.txt" 2>&1 || true
# Current logcat is optional; benchmark files and saved camera settings are mandatory.
adb -s "$device" logcat -d -v threadtime > "$out/logcat.txt" 2> "$out/logcat-error.txt" || true
{
  git rev-parse HEAD
  git status --short
  git diff --stat
  git diff -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen
} > "$out/checkout.txt"
if ! python3 "$(dirname "$0")/hive-benchmark-analyze.py" "$out/pollen-hive" --out "$out/analysis" > "$out/analysis-output.txt" 2>&1; then
  echo "Analysis could not finish; raw logs are preserved. See analysis-output.txt." >&2
fi
python3 "$(dirname "$0")/hive-auto-analyze.py" "$out/pollen-hive" > "$out/auto-summary.md" 2> "$out/auto-analysis-error.txt" || true
echo "Creating zip..."
(cd "$(dirname "$out")" && zip -qr "$(basename "$out").zip" "$(basename "$out")")
# Show the results now, so the next test can be chosen without opening any files.
cat "$out/auto-summary.md"
printf "\nCamera benchmark summary, if present: %s/analysis/summary.md\n" "$out"
printf '\nDownload complete: %s.zip (%s)\nInclude run notes and video; pipeline exports only if changed.\n' "$out" "$(du -h "$out.zip" | cut -f1)"
