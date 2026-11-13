#!/usr/bin/env bash
# Computer tests for the PollenBot code, plus the field-route checker (needs Java 17+ and python3).
# Students: the same tests run from Android Studio (TeamCode/src/test/.../pollen/PollenTests.java).
set -euo pipefail
repo_dir="$(cd "$(dirname "$0")/.." && pwd)"
test_build_dir="$(mktemp -d)"
trap 'rm -rf "$test_build_dir"' EXIT
src="$repo_dir/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen"
tests="$repo_dir/TeamCode/src/test/java/org/firstinspires/ftc/teamcode/pollen"
pkg=org.firstinspires.ftc.teamcode.pollen
# PollenTests.java (the JUnit wrapper for Android Studio) is left out: it needs JUnit, which
# Gradle supplies. Everything else here is plain Java.
javac --release 8 -d "$test_build_dir" "$src/PollenSequence.java" "$src/PollenFeedRule.java" "$src/PollenConfig.java" "$src/PollenParkTuning.java" "$src/PollenShootTuning.java" "$src/PollenDriveCalibration.java" "$src/PollenFieldGeometry.java" "$src/PollenTeleOpTuning.java" "$src/PollenOdometryDrive.java" "$src/PollenOdoParkTuning.java" "$src/PollenAutoLog.java" "$src/PollenFacing.java" "$src/IsHiveReadyForShot.java" "$src/PollenHiveCalibration.java" "$src/PollenHiveCalibrationSession.java" "$src/PollenHiveSettings.java" "$src/PollenHiveRoute.java" "$src/PollenHiveMotion.java" "$src/PollenHiveAuto.java" "$src/PollenHiveBenchmarkPlan.java" \
  "$tests/PollenSequenceTest.java" \
  "$tests/PollenParkTuningTest.java" \
  "$tests/PollenShootFirstTest.java" \
  "$tests/PollenMatSupportTest.java" \
  "$tests/PollenTeleOpTuningTest.java" \
  "$tests/PollenLoopTimingTest.java" \
  "$tests/PollenPresetExport.java" \
  "$tests/PollenOdometryTest.java" \
  "$tests/PollenHiveTest.java" \
  "$tests/PollenHiveBenchmarkTest.java"
java -cp "$test_build_dir" $pkg.PollenSequenceTest
java -cp "$test_build_dir" $pkg.PollenParkTuningTest
java -cp "$test_build_dir" $pkg.PollenShootFirstTest
java -cp "$test_build_dir" $pkg.PollenMatSupportTest
java -cp "$test_build_dir" $pkg.PollenTeleOpTuningTest
java -cp "$test_build_dir" $pkg.PollenLoopTimingTest
java -cp "$test_build_dir" $pkg.PollenOdometryTest
java -cp "$test_build_dir" $pkg.PollenHiveTest
java -cp "$test_build_dir" $pkg.PollenHiveBenchmarkTest
# Check the routes the Java presets actually drive against the CAD field model.
java -cp "$test_build_dir" $pkg.PollenPresetExport > "$test_build_dir/presets.json"
python3 "$repo_dir/tools/check-pollen-field-routes.py" --presets "$test_build_dir/presets.json"
# The promote tool: saved Hub changes become the code defaults (on a scratch copy).
python3 "$repo_dir/tools/test-hive-promote.py"

python3 "$repo_dir/tools/test-hive-benchmark.py"
