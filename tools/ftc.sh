#!/usr/bin/env bash
# Stable entry point for humans, Claude Code and Codex local sessions.
set -euo pipefail
source "$(dirname "$0")/ftc-common.sh"
usage() {
  cat <<'TXT'
Usage: bash tools/ftc.sh COMMAND [arguments]
  doctor                     Inspect checkout, Java, SDK, adb and device list
  connect [SERIAL]            Check, connect, restart ADB once if needed, retry
  adb ARGS...                Run adb -s FTC_ROBOT_SERIAL ARGS (no auto-reconnect)
  test                       Run the Java/Python computer tests and tool tests
  build                      Build the debug APK (no robot needed)
  gradle ARGS...              Run Gradle with an explicit JDK and SDK discovery
  deploy [SERIAL]             Build, connect, install that APK on one selected hub
  install [SERIAL]            Install the existing APK; DOES NOT rebuild
  logs NEW_FOLDER [SERIAL]    Download hive data, SDK logs, configs and analysis zip
  storage [SERIAL]            Show the hub's free space and recording folder sizes (read-only)
  prune FOLDER [--delete]     Compare the hub's recordings with a downloaded FOLDER; with
                             --delete, remove from the hub only files identical to the download
  patch FILE.patch           Apply a git-format-patch on clean pollen-hive-ready
  help                       Show this help

Default target: FTC_ROBOT_SERIAL or 192.168.43.1:5555. Check the Wi-Fi network!
Overrides: FTC_ADB, FTC_JAVA_HOME, ANDROID_HOME, FTC_OFFLINE=1,
           FTC_ADB_RESTART=0, FTC_ADB_TIMEOUT (seconds).
Install/deploy restart the RC app.
No command starts an OpMode. See docs/DESKTOP_TOOLS.md.
TXT
}
no_args() { [ "$#" -eq 0 ] || { usage >&2; exit 2; }; }
one_optional() { [ "$#" -le 1 ] || { usage >&2; exit 2; }; }

command_name=${1:-help}
[ "$#" -eq 0 ] || shift
case "$command_name" in
  help|-h|--help) usage ;;
  doctor)
    no_args "$@"
    cd "$FTC_ROOT"
    printf 'Checkout: %s\nBranch: ' "$FTC_ROOT"
    git branch --show-current
    git log -1 --format='%h %s'
    git status --short
    git diff --check HEAD
    failures=0
    if ftc_java; then printf 'JDK: %s\n' "$JAVA_HOME"; java -version; else failures=1; fi
    if ftc_sdk; then printf 'SDK: %s\n' "${ANDROID_HOME:-see local.properties}"; else failures=1; fi
    if ftc_find_adb; then
      printf 'ADB: %s\n' "$FTC_ADB"
      ftc_adb version || failures=1
      ftc_adb devices -l || failures=1
    else failures=1; fi
    ftc_device
    printf 'Target: %s (doctor does not connect or restart ADB)\n' "$FTC_DEVICE"
    printf 'Python: '; python3 --version || failures=1
    command -v zip >/dev/null || { echo 'Missing zip (needed for logs).' >&2; failures=1; }
    exit "$failures" ;;
  connect) one_optional "$@"; ftc_connect "${1:-}" ;;
  adb)
    [ "$#" -gt 0 ] || { usage >&2; exit 2; }
    ftc_find_adb; ftc_device
    # Prevent a second selector overriding the explicitly configured robot.
    for arg in "$@"; do
      case "$arg" in -s|-d|-e|-t) ftc_error 'Set FTC_ROBOT_SERIAL instead of passing another device selector.'; exit 2 ;; esac
    done
    ftc_adb -s "$FTC_DEVICE" "$@" ;;
  test)
    no_args "$@"
    ftc_java; ftc_python
    cd "$FTC_ROOT"
    bash tools/test-pollen-sequence.sh
    python3 tools/test-ftc-tools.py ;;
  build) no_args "$@"; ftc_gradle :TeamCode:assembleDebug ;;
  gradle) [ "$#" -gt 0 ] || { usage >&2; exit 2; }; ftc_gradle "$@" ;;
  deploy|install)
    one_optional "$@"
    ftc_device "${1:-}"
    case "$FTC_DEVICE" in emulator-*) ftc_error 'Refusing robot deployment to an emulator.'; exit 1 ;; esac
    echo 'Install will restart the Robot Controller app.'
    if [ "$command_name" = deploy ]; then ftc_gradle :TeamCode:assembleDebug; fi
    apk="$FTC_ROOT/TeamCode/build/outputs/apk/debug/TeamCode-debug.apk"
    [ -f "$apk" ] || { ftc_error 'APK missing. Run build (on internet Wi-Fi) first.'; exit 1; }
    if [ "$command_name" = install ]; then echo 'Using existing APK; source changes since its build are NOT included.'; fi
    ls -l "$apk"
    ftc_connect "$FTC_DEVICE"
    FTC_ADB_TIMEOUT=${FTC_ADB_TIMEOUT:-180} ftc_adb -s "$FTC_DEVICE" install -r "$apk"
    echo 'Install succeeded. On Driver Hub INIT, verify the expected Build value. No OpMode was started.' ;;
  logs)
    [ "$#" -ge 1 ] && [ "$#" -le 2 ] || { usage >&2; exit 2; }
    exec bash "$FTC_ROOT/tools/hive-benchmark-download.sh" "$@" ;;
  storage)
    one_optional "$@"
    ftc_python
    ftc_connect "${1:-}"
    python3 "$FTC_ROOT/tools/hive-storage.py" report ;;
  prune)
    # Removal is opt-in and checked file by file; see docs/POLLEN_LOGGING.md.
    [ "$#" -ge 1 ] && [ "$#" -le 2 ] || { usage >&2; exit 2; }
    [ "$#" -eq 1 ] || [ "$2" = --delete ] || { usage >&2; exit 2; }
    [ -d "$1" ] || { ftc_error "Not a downloaded folder: $1"; exit 1; }
    ftc_python
    ftc_connect
    echo 'Run prune only after the OpMode is STOPPED: a recording still being written must stay.'
    python3 "$FTC_ROOT/tools/hive-storage.py" prune "$@" ;;
  patch)
    [ "$#" -eq 1 ] || { usage >&2; exit 2; }
    ftc_python
    patch_file=$(python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "$1")
    [ -f "$patch_file" ] || { ftc_error "Missing patch: $patch_file"; exit 1; }
    cd "$FTC_ROOT"
    [ "$(git branch --show-current)" = pollen-hive-ready ] || { ftc_error 'Patch command requires pollen-hive-ready.'; exit 1; }
    [ -z "$(git status --porcelain)" ] || { ftc_error 'Review and commit local work first. Keep the downloaded patch outside the checkout.'; exit 1; }
    [ ! -d "$(git rev-parse --git-path rebase-apply)" ] && [ ! -d "$(git rev-parse --git-path rebase-merge)" ] || {
      ftc_error 'Finish the existing git am/rebase operation first.'; exit 1;
    }
    # A mail patch (git format-patch) has an Author header; a plain git diff does not.
    case "$(git mailinfo /dev/null /dev/null < "$patch_file")" in
      'Author: '?*) ;;
      *) ftc_error 'Not a mail-format patch. Create it with git format-patch, not git diff.'; exit 1 ;;
    esac
    git apply --check "$patch_file"
    backup="backup/pollen-before-patch-$(date +%Y%m%d-%H%M%S)-$$"
    git branch "$backup"
    printf 'Backup branch: %s\n' "$backup"
    git am "$patch_file"
    echo 'Patch committed locally. Run test and build before deploying or pushing.' ;;
  *) usage >&2; exit 2 ;;
esac
