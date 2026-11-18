# Desktop tools for PollenBot

Use these scripts from a **local checkout on the laptop connected to the robot**.
They work with Claude Code's local desktop sessions, Codex local sessions, or a terminal.
The command is `adb devices -l` (plural `devices`; `adb`, not `adp`). You no longer
need an `adb` alias or an `ftcgradle` shell function for the workflows below.

## Open the project in either desktop app

1. Install Android Studio and the project's Android SDK platform (API 36), SDK Build-Tools
   and Platform-Tools. Install Python 3; the log bundle also needs `zip`. See
   [student setup](ROBOT_CONNECTION.md). Do the initial Android build while you have internet.
2. In Claude's desktop coding interface, select this local repository. In Codex, open
   this repository in a local environment. A cloud session cannot reach your laptop's
   Wi-Fi robot. These are ordinary local shell scripts; no MCP server is needed.
3. Confirm the actual checkout with `git branch --show-current` and `git log -1 --oneline`.
   For this work use `pollen-hive-ready`. If the app made a separate worktree, verify its
   branch and edits before building; `local.properties` and built APKs may not be there.
4. Start a new session after `AGENTS.md` changes. `AGENTS.md` contains the shared
   instructions; `CLAUDE.md` imports it. A local session edits the checkout, runs the
   tests/builds and uses these scripts itself; it does not need patch files. A useful
   first request is: “Check this checkout and setup; run the computer tests and build.
   Don't install until I ask.”
5. Grant the application's normal requested project/command permissions. For robot work,
   the host process executing ADB also needs local-network access. If the Mac blocks
   access, inspect System Settings → Privacy & Security → Local Network for that app
   or Terminal, then restart the affected app after changing the permission. Scripts
   cannot grant these permissions or bypass an app sandbox.

The tools support macOS Bash 3.2 and Linux Bash. Native Windows PowerShell users should
use [ROBOT_CONNECTION.md](ROBOT_CONNECTION.md); these Bash scripts are not a promise of
WSL-to-hub network support. Personal SSH accounts and private key paths stay outside the
team repository. Do not have Claude and Codex edit the same worktree simultaneously.

Official references: [Claude desktop](https://code.claude.com/docs/en/desktop),
[Claude instruction imports](https://code.claude.com/docs/en/memory),
[Codex local environments](https://learn.chatgpt.com/docs/environments/local-environment),
[Codex AGENTS.md](https://learn.chatgpt.com/docs/agent-configuration/agents-md),
[Android ADB](https://developer.android.com/tools/adb).

## Daily commands

Run from the repository root. Scripts resolve paths relative to themselves, so an agent
can also call `bash /absolute/path/to/FtcRobotController/tools/ftc.sh ...` from elsewhere.

| Job | Command | Result |
| --- | --- | --- |
| Inspect setup | `bash tools/ftc.sh doctor` | Checkout/branch/status, Java, SDK discovery, ADB path/version, device list; nonzero for missing prerequisites |
| Reconnect | `bash tools/ftc.sh connect` | Check the target, attempt connection, restart ADB once if necessary, retry and verify |
| Computer tests | `bash tools/ftc.sh test` | Existing Java/Python suite plus desktop-tool regression tests; no robot |
| Android unit tests and build | `bash tools/ftc.sh gradle :TeamCode:testDebugUnitTest :TeamCode:assembleDebug` | Gradle uses an explicit full JDK; no robot |
| Build | `bash tools/ftc.sh build` | Fresh debug APK; no install |
| Deploy | `bash tools/ftc.sh deploy` | Build, then connect and install only on the selected hub |
| Install cached build | `bash tools/ftc.sh install` | Install existing APK without building; prints its timestamp |
| Save stopped session | `bash tools/ftc.sh logs "$HOME/Desktop/hive-session-$(date +%Y%m%d-%H%M%S)"` | New directory and ZIP, including raw data and analysis |
| Hub storage | `bash tools/ftc.sh storage` | Read-only: free space and each recording folder against its budget |
| Free hub storage | `bash tools/ftc.sh prune FOLDER` then `... prune FOLDER --delete` | Lists, then removes from the hub only recordings whose checksum matches the copy in a downloaded FOLDER. Settings stay. See [recordings and storage](POLLEN_LOGGING.md) |
| Rehearsals to waypoints | `python3 tools/teleop-rehearsal.py FOLDER --out FOLDER/rehearsals` | Draft autonomous waypoints from the Circle/Triangle marks in TeleOp recordings; `logs` already runs it |
| Inspect hub model | `bash tools/ftc.sh adb shell getprop ro.product.model` | Explicitly targeted ADB call; no reconnect |
| Stop a stuck Gradle daemon | `bash tools/ftc.sh gradle --stop` | Stops project Gradle daemons using the selected JDK |
| Apply a later mail patch | `bash tools/ftc.sh patch "$HOME/Downloads/change.patch"` | Clean-tree/branch checks, backup branch, `git am`; no tests or push |

`doctor` may start ADB's server when listing devices, but does not connect, restart it,
install or change files. An absent robot is normal at home and is not a setup failure.
Doctor checks SDK discovery, not whether every SDK package or dependency is installed;
only the Android build verifies those. The raw `adb`/`gradle` passthrough commands run
exactly what you request; e.g. a direct Gradle `installDebug` is outside the deployment
helper's explicit device selection. Prefer `deploy`/`install` for robot installation.

### Reconnect behavior

Default target: `192.168.43.1:5555`, the address used on this team's Hub Wi-Fi. It is a
configurable network endpoint, not a unique robot identity. Join the intended robot's
network, especially when other team robots are nearby.

1. Show `adb devices -l` and check `adb -s TARGET get-state`.
2. If it already says `device`, finish without disturbing the server.
3. Otherwise, for an IP:port target, try `adb connect TARGET` and check again.
4. If still not online, disconnect that target, run `adb kill-server`, `adb start-server`,
   reconnect and check once more. This resets the laptop's shared ADB server and affects
   Android Studio and other ADB clients. It never kills an emulator or Java process.
5. Fail with a troubleshooting message if the selected target still is not `device`.
   A successful-looking `connect` message alone does not count. An explicitly reported
   `unauthorized` state stops for authorization resolution.

ADB commands time out after 20 seconds each; install/log transfers allow 180 seconds per
command by default. There is no indefinite retry loop. A USB serial can be selected
explicitly, but is never passed to `adb connect` as though it were an IP address.

```bash
# One invocation, another hub endpoint:
bash tools/ftc.sh connect 192.168.49.1:5555

# Persist for commands in this terminal or the app's local environment:
export FTC_ROBOT_SERIAL=192.168.43.1:5555

# Check/attempt reconnect without restarting the shared ADB server:
FTC_ADB_RESTART=0 bash tools/ftc.sh connect
```

After a failed retry: check the Wi-Fi network and hub power, wait for boot, inspect VPN
routing and macOS Local Network permissions. If another app keeps restarting a conflicting
ADB, close it deliberately and retry. Do not automatically kill every emulator/Gradle
process or power-cycle a moving robot. ADB online does not prove the RC app is initialized.

### Build on internet Wi-Fi, install on robot Wi-Fi

```bash
# While the laptop has internet:
bash tools/ftc.sh test
bash tools/ftc.sh gradle :TeamCode:testDebugUnitTest :TeamCode:assembleDebug

# Join the intended robot's Wi-Fi.
bash tools/ftc.sh install
```

`install` deliberately uses the existing `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.
Rebuild if source changed. `deploy` always builds first and stops on a build failure,
so it cannot silently install an old APK after a failed build. For cached dependencies:

```bash
FTC_OFFLINE=1 bash tools/ftc.sh deploy
```

Offline mode works only after the Gradle distribution and dependencies have been cached.
Installation restarts the RC app. The operator controls INIT/START and checks the Build line on the
Driver Hub against the expected build (`pollen-hive-v10` since TeleOp recording was added). Source Git
HEAD is not proof of what is installed. Signing errors are reported; nothing is uninstalled.

### Save and analyze data

After STOP, run `logs`. It reuses `hive-benchmark-download.sh` with the same connection
recovery as `connect`. Existing direct downloader commands still work.

The archive contains `/sdcard/FIRST/pollen-hive` (required), available SDK logs, hardware
XML, match logs, logcat, installed-package info, storage usage, and checkout context.
The context now records the selected serial and the actual script checkout even when
called from elsewhere. It is labelled checkout context, not proof of installed source.
Existing output directories or ZIPs are refused. Nothing is deleted from the hub.
Optional pull errors remain in `optional-pulls.txt`; analysis failures preserve raw data.
Missing mandatory hive data fails the command. Preserve a partial directory on failure;
retry into a new destination. Review logs/configuration before sharing outside the team.

Repeated offline workflows already have tools; use them directly rather than wrapping
another copy of their logic:

```bash
python3 tools/hive-auto-analyze.py /path/to/session/pollen-hive
python3 tools/hive-benchmark-analyze.py /path/to/session/pollen-hive --out /path/to/new-analysis
python3 tools/hive-pipeline-backup.py --host limelight.local --out /path/to/new-pipeline-backup
python3 tools/hive-camera-record.py --help
python3 tools/hive-promote-defaults.py --help
```

Pipeline export requires the camera to be reachable from this laptop. It reads saved
camera settings; it does not tune pipelines. Promotion changes source defaults and
requires an explicit decision to adopt the measured settings; downloading never promotes.
For paper-target limits, tests and run procedures use the hive guides in the docs index.

## Environment overrides (no global configuration changes)

| Variable | Meaning |
| --- | --- |
| `FTC_ADB` | Absolute path to an executable ADB binary |
| `FTC_ROBOT_SERIAL` | Target serial/IP:port; explicit command argument wins |
| `FTC_JAVA_HOME` | Full JDK home (17+); highest priority |
| `ANDROID_HOME` / `ANDROID_SDK_ROOT` | SDK directory; otherwise Gradle can use `local.properties`, then tools try standard Mac/Linux SDK locations |
| `FTC_OFFLINE=1` | Add Gradle `--offline` |
| `FTC_ADB_RESTART=0` | Disable the one-time ADB-server restart |
| `FTC_ADB_TIMEOUT` | Per-command timeout in seconds, greater than 0, at most 3600 |

ADB discovery: `FTC_ADB`, executable on PATH, SDK environment directories, standard
Mac/Linux SDK paths. A custom SDK recorded only in `local.properties` still needs
`FTC_ADB` or a matching SDK environment variable for ADB discovery. JDK discovery:
`FTC_JAVA_HOME`, Android Studio's standard Mac bundled JDK, `JAVA_HOME`, macOS
`java_home`, then PATH's `javac`. An invalid explicit override fails instead of being ignored.
The Gradle wrapper also receives `-Dorg.gradle.java.home` so a stale default JVM does not win.
Set overrides in the desktop session's environment if needed; GUI apps need not inherit
Terminal aliases or `.zshrc`. Existing personal `ftcgradle` functions remain untouched.

## Patch and validation workflow

Local desktop sessions change files directly and do not need patches. `ftc.sh patch` is
for applying a mail patch produced by a web/cloud session that could not reach this
laptop. Keep patches outside the checkout. That helper requires
`pollen-hive-ready`, a clean tree, no in-progress am/rebase, and a successful application
precheck. It creates a backup branch, then commits the patch locally via `git am`.
It never stashes, resets, pulls, pushes or deploys. Inspect failures before deciding on
`git am --continue` or `git am --abort`. It accepts mail-format patches, not plain diffs.

Before sharing script changes:

```bash
python3 tools/test-ftc-tools.py
python3 tools/test-hive-download.py
bash -n tools/ftc.sh tools/ftc-common.sh tools/hive-benchmark-download.sh
git diff --check HEAD
```

The tool tests simulate ADB/JDK/Gradle and do not touch real devices. They verify bounded
recovery, explicit device selection, failures, paths with spaces, discovery and build/
install separation. They do not certify Wi-Fi, desktop-app permissions or physical motion.
