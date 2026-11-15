#!/usr/bin/env bash
# Shared by the command-line tools. Compatible with macOS Bash 3.2.
# Source this file; it does not change global Git settings or shell startup files.
FTC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ftc_error() { printf 'FTC: %s\n' "$*" >&2; return 1; }
ftc_python() { command -v python3 >/dev/null || ftc_error 'Install Python 3 first.'; }

ftc_find_adb() {
  local candidate
  if [ -n "${FTC_ADB:-}" ]; then
    [ -x "$FTC_ADB" ] || { ftc_error "FTC_ADB is not executable: $FTC_ADB"; return 1; }
  else
    FTC_ADB=$(type -P adb || true)
    if [ -z "$FTC_ADB" ]; then
      for candidate in "${ANDROID_HOME:-}/platform-tools/adb" \
        "${ANDROID_SDK_ROOT:-}/platform-tools/adb" \
        "$HOME/Library/Android/sdk/platform-tools/adb" "$HOME/Android/Sdk/platform-tools/adb"; do
        if [ -x "$candidate" ]; then FTC_ADB=$candidate; break; fi
      done
    fi
    [ -n "$FTC_ADB" ] || { ftc_error 'adb not found. Install SDK Platform-Tools or set FTC_ADB to the full binary path (aliases do not work in scripts).'; return 1; }
  fi
  export FTC_ADB
  ftc_python
}

# A hung connect must not leave a desktop agent waiting indefinitely. Python's
# subprocess timeout works on macOS without GNU timeout. Pull/install use a longer limit.
ftc_adb() {
  python3 - "$FTC_ADB" "${FTC_ADB_TIMEOUT:-20}" "$@" <<'PY'
import subprocess, sys
try:
    timeout = float(sys.argv[2])
    if not 0 < timeout <= 3600:
        raise ValueError()
except ValueError:
    sys.exit('FTC_ADB_TIMEOUT must be > 0 and <= 3600 seconds')
try:
    sys.exit(subprocess.run([sys.argv[1], *sys.argv[3:]], timeout=timeout).returncode)
except subprocess.TimeoutExpired:
    print('ADB timed out after %g seconds' % timeout, file=sys.stderr)
    sys.exit(124)
PY
}

ftc_device() {
  FTC_DEVICE=${1:-${FTC_ROBOT_SERIAL:-192.168.43.1:5555}}
  case "$FTC_DEVICE" in ''|-*|*[[:space:]]*) ftc_error 'Invalid robot serial.'; return 1 ;; esac
  export FTC_DEVICE
}

ftc_online() {
  [ "$(ftc_adb -s "$FTC_DEVICE" get-state 2>/dev/null | tr -d '\r')" = device ]
}

ftc_connect() {
  local state
  ftc_find_adb || return 1
  ftc_device "${1:-}" || return 1
  printf 'ADB: %s\nTarget: %s\n' "$FTC_ADB" "$FTC_DEVICE"
  ftc_adb devices -l || true
  if ftc_online; then printf 'Connected: %s\n' "$FTC_DEVICE"; return 0; fi
  state=$(ftc_adb -s "$FTC_DEVICE" get-state 2>/dev/null || true)
  if [ "$state" = unauthorized ]; then
    ftc_error 'Hub is unauthorized; resolve device authorization before retrying.'; return 1
  fi
  case "$FTC_DEVICE" in
    *:*)
      ftc_adb connect "$FTC_DEVICE" || true
      if ftc_online; then printf 'Connected: %s\n' "$FTC_DEVICE"; return 0; fi ;;
  esac
  # The server is shared by Android Studio and other adb clients on this laptop.
  # One restart only; never pkill Java, Android Studio, or an emulator.
  if [ "${FTC_ADB_RESTART:-1}" != 0 ]; then
    echo 'Normal connection failed; restarting the shared local ADB server once.'
    case "$FTC_DEVICE" in *:*) ftc_adb disconnect "$FTC_DEVICE" || true ;; esac
    ftc_adb kill-server || return 1
    ftc_adb start-server || return 1
    case "$FTC_DEVICE" in *:*) ftc_adb connect "$FTC_DEVICE" || true ;; esac
  fi
  ftc_adb devices -l || true
  if ftc_online; then printf 'Connected: %s\n' "$FTC_DEVICE"; return 0; fi
  ftc_error "Hub $FTC_DEVICE is not online. Join the intended Hub Wi-Fi, wait for boot, check VPN/routing and the desktop app's macOS Local Network permission. See docs/DESKTOP_TOOLS.md."
}

ftc_java() {
  local candidate version major
  if [ -n "${FTC_JAVA_HOME:-}" ]; then
    candidate=$FTC_JAVA_HOME
  elif [ -x '/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin/javac' ]; then
    candidate='/Applications/Android Studio.app/Contents/jbr/Contents/Home'
  elif [ -n "${JAVA_HOME:-}" ]; then
    candidate=$JAVA_HOME
  elif [ -x /usr/libexec/java_home ]; then
    candidate=$(/usr/libexec/java_home -v 17+) || return 1
  elif type -P javac >/dev/null; then
    ftc_python || return 1
    candidate=$(python3 -c 'import os,shutil; print(os.path.dirname(os.path.dirname(os.path.realpath(shutil.which("javac")))))')
  else
    ftc_error 'JDK 17+ required. Install Android Studio or set FTC_JAVA_HOME (a JRE is insufficient).'; return 1
  fi
  [ -x "$candidate/bin/java" ] && [ -x "$candidate/bin/javac" ] || {
    ftc_error "Not a full JDK: $candidate"; return 1;
  }
  version=$("$candidate/bin/javac" -version 2>&1) || return 1
  major=${version#javac }; major=${major%%.*}; major=${major%%-*}
  case "$major" in ''|*[!0-9]*) ftc_error "Cannot parse javac version: $version"; return 1 ;; esac
  [ "$major" -ge 17 ] || { ftc_error "JDK 17+ required; found $version"; return 1; }
  export JAVA_HOME="$candidate" PATH="$candidate/bin:$PATH"
}

ftc_sdk() {
  local candidate
  if [ -n "${ANDROID_HOME:-}" ]; then candidate=$ANDROID_HOME
  elif [ -n "${ANDROID_SDK_ROOT:-}" ]; then candidate=$ANDROID_SDK_ROOT
  elif [ -f "$FTC_ROOT/local.properties" ]; then return 0  # Gradle owns Java-properties parsing.
  elif [ -d "$HOME/Library/Android/sdk" ]; then candidate="$HOME/Library/Android/sdk"
  elif [ -d "$HOME/Android/Sdk" ]; then candidate="$HOME/Android/Sdk"
  else ftc_error 'Android SDK missing. Open the project in Android Studio or set ANDROID_HOME.'; return 1
  fi
  [ -d "$candidate" ] || { ftc_error "SDK directory missing: $candidate"; return 1; }
  export ANDROID_HOME="$candidate"
}

ftc_gradle() (
  ftc_java || exit 1
  ftc_sdk || exit 1
  cd "$FTC_ROOT" || exit 1
  if [ "${FTC_OFFLINE:-0}" = 1 ]; then set -- --offline "$@"; fi
  bash ./gradlew "-Dorg.gradle.java.home=$JAVA_HOME" "$@"
)
