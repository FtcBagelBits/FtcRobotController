#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
test_build_dir="$(mktemp -d)"
trap 'rm -rf "$test_build_dir"' EXIT
src="$repo_dir/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen"
javac --release 8 -d "$test_build_dir" "$src/PollenSequence.java" "$src/PollenConfig.java" "$src/PollenParkTuning.java" "$src/PollenShootTuning.java" "$src/PollenDriveCalibration.java" "$src/PollenFieldGeometry.java" "$src/PollenTeleOpTuning.java" \
  "$repo_dir/tools/pollen-tests/PollenSequenceTest.java" \
  "$repo_dir/tools/pollen-tests/PollenParkTuningTest.java" \
  "$repo_dir/tools/pollen-tests/PollenShootFirstTest.java" \
  "$repo_dir/tools/pollen-tests/PollenMatSupportTest.java" \
  "$repo_dir/tools/pollen-tests/PollenTeleOpTuningTest.java" \
  "$repo_dir/tools/pollen-tests/PollenLoopTimingTest.java" \
  "$repo_dir/tools/pollen-tests/PollenPresetExport.java"
java -cp "$test_build_dir" PollenSequenceTest
java -cp "$test_build_dir" PollenParkTuningTest
java -cp "$test_build_dir" PollenShootFirstTest
java -cp "$test_build_dir" PollenMatSupportTest
java -cp "$test_build_dir" PollenTeleOpTuningTest
java -cp "$test_build_dir" PollenLoopTimingTest
# Check the routes the Java presets actually drive against the CAD field model.
java -cp "$test_build_dir" PollenPresetExport > "$test_build_dir/presets.json"
python3 "$repo_dir/tools/check-pollen-field-routes.py" --presets "$test_build_dir/presets.json"
