#!/usr/bin/env bash
# Run after STOP; explicit device selection avoids accidentally pulling an emulator.
set -euo pipefail
if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo 'Usage: bash tools/hive-benchmark-download.sh NEW_OUTPUT_FOLDER [ADB_DEVICE]' >&2
  exit 2
fi
out=$1
source "$(dirname "$0")/ftc-common.sh"
ftc_device "${2:-}"
device=$FTC_DEVICE
if [ -e "$out" ] || [ -e "$out.zip" ]; then echo 'Choose a new output folder (no overwrites).' >&2; exit 1; fi
command -v zip >/dev/null
ftc_connect "$device"
# Large log pulls can take longer than the connection probe.
export FTC_ADB_TIMEOUT=${FTC_ADB_TIMEOUT:-180}
mkdir -p "$out"
echo "Downloading saved settings, reports, benchmarks, auto runs and TeleOp recordings..."
ftc_adb -s "$device" pull /sdcard/FIRST/pollen-hive "$out/pollen-hive"
echo 'Downloading SDK logs and storage status...'
ftc_adb -s "$device" shell 'df -h /sdcard; du -k /sdcard/FIRST/pollen-hive' > "$out/storage.txt" 2>&1 || true
mkdir -p "$out/robot-logs" "$out/configs"
# SDK rotations and hardware XML help diagnose resets, configuration and motor mapping.
# Optional pulls never discard the mandatory hive folder if a file is absent.
while IFS= read -r name; do
  case "$name" in robotControllerLog*)
    ftc_adb -s "$device" pull "/sdcard/$name" "$out/robot-logs/" >> "$out/optional-pulls.txt" 2>&1 || true ;;
  esac
done < <(ftc_adb -s "$device" shell 'ls /sdcard' 2>/dev/null | tr -d '\r')
while IFS= read -r name; do
  case "$name" in *.xml)
    ftc_adb -s "$device" pull "/sdcard/FIRST/$name" "$out/configs/" >> "$out/optional-pulls.txt" 2>&1 || true ;;
  esac
done < <(ftc_adb -s "$device" shell 'ls /sdcard/FIRST' 2>/dev/null | tr -d '\r')
ftc_adb -s "$device" pull /sdcard/FIRST/matchlogs "$out/robot-logs/matchlogs" >> "$out/optional-pulls.txt" 2>&1 || true
ftc_adb -s "$device" shell dumpsys package com.qualcomm.ftcrobotcontroller > "$out/installed-app.txt" 2>&1 || true
# Current logcat is optional; benchmark files and saved camera settings are mandatory.
ftc_adb -s "$device" logcat -d -v threadtime > "$out/logcat.txt" 2> "$out/logcat-error.txt" || true
(
  printf 'ADB target: %s\nCheckout: %s\n' "$device" "$FTC_ROOT"
  cd "$FTC_ROOT"
  git rev-parse HEAD
  git status --short
  git diff --stat
  git diff -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen
) > "$out/checkout.txt"
if ! python3 "$(dirname "$0")/hive-benchmark-analyze.py" "$out/pollen-hive" --out "$out/analysis" > "$out/analysis-output.txt" 2>&1; then
  echo "Analysis could not finish; raw logs are preserved. See analysis-output.txt." >&2
fi
python3 "$(dirname "$0")/hive-auto-analyze.py" "$out/pollen-hive" > "$out/auto-summary.md" 2> "$out/auto-analysis-error.txt" || true
# TeleOp recordings: each marked rehearsal becomes draft waypoints (docs/POLLEN_LOGGING.md).
python3 "$(dirname "$0")/teleop-rehearsal.py" "$out/pollen-hive" --out "$out/rehearsals" > "$out/teleop-summary.md" 2> "$out/teleop-analysis-error.txt" || true
echo "Creating zip..."
(cd "$(dirname "$out")" && zip -qr "$(basename "$out").zip" "$(basename "$out")")
# Show the results now, so the next test can be chosen without opening any files.
cat "$out/auto-summary.md"
printf "\nCamera benchmark summary, if present: %s/analysis/summary.md\n" "$out"
printf 'TeleOp recordings and rehearsals: %s/teleop-summary.md\n' "$out"
printf '\nDownload complete: %s.zip (%s)\nInclude run notes and video; pipeline exports only if changed.\n' "$out" "$(du -h "$out.zip" | cut -f1)"
