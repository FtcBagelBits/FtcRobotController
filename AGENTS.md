# FTC Team 31452 — shared agent instructions

This file is authoritative for repository workflow. `CLAUDE.md` imports it for Claude;
Codex reads it directly. Use this exact uppercase filename, not `Agents.md`. Keep
machine-specific credentials, SSH identities and personal workflows outside this repo.

**Local desktop sessions are the normal way of working.** When the agent runs on the
laptop's checkout (Claude Code desktop/CLI, Codex local), edit files on disk directly, run
the tests and builds yourself, and use the `tools/ftc.sh` commands for robot work. Do not
hand the human patch files or commands to run that you can run yourself. Mail patches are
only for web/cloud sessions that cannot reach this laptop, or when the user asks for one.

## Start with the actual checkout

- Inspect `pwd`, `git status --short`, `git branch --show-current`, and `git log -1 --oneline`.
  This work targets `pollen-hive-ready`; stay on the user's requested branch. Do not
  create or switch a branch merely because older instructions used `main` or `pollen-auto-prep`.
- Run `bash tools/ftc.sh doctor` for environment trouble. A cloud checkout is not the
  user's Mac, robot, installed APK, or uncommitted local changes. Say where checks ran.
- Read [Desktop tools](docs/DESKTOP_TOOLS.md) before connection, build, installation,
  patch or download work. Use its scripts instead of reconstructing aliases or reset recipes.
- [Code reference](docs/POLLEN_CODE_REFERENCE.md) preserves hardware names, facing
  transforms, legacy modes, feed rules and odometry/Pedro context. Read relevant sections
  before editing robot code. [Docs index](docs/README.md) points to operator instructions.
- [Recordings and storage](docs/POLLEN_LOGGING.md) says what each mode records, the folder
  budgets on the hub and the download-then-prune lifecycle. Read it before changing logging.
- On this branch read [hive testing](docs/POLLEN_HIVE_TESTING.md),
  [benchmark](docs/POLLEN_HIVE_BENCHMARK.md) and
  [diagnostics](docs/POLLEN_HIVE_DIAGNOSTICS.md) for hive work. Source code owns current
  defaults. `PollenConfig.BUILD` is the build label; never infer the installed label from Git.

## Reusable commands

From the repository root (the scripts also resolve their own root):

| Intent | Command |
| --- | --- |
| Check setup and checkout | `bash tools/ftc.sh doctor` |
| Reconnect to the intended hub | `bash tools/ftc.sh connect` |
| Computer tests | `bash tools/ftc.sh test` |
| Android unit tests and APK | `bash tools/ftc.sh gradle :TeamCode:testDebugUnitTest :TeamCode:assembleDebug` |
| Build only | `bash tools/ftc.sh build` |
| Build and install after operator STOP | `bash tools/ftc.sh deploy` |
| Install an already built APK after switching to hub Wi-Fi | `bash tools/ftc.sh install` |
| Download and analyze a stopped session | `bash tools/ftc.sh logs "$HOME/Desktop/hive-session-$(date +%Y%m%d-%H%M%S)"` |
| Hub free space and recording folder sizes (read-only) | `bash tools/ftc.sh storage` |
| List hub recordings that match a download | `bash tools/ftc.sh prune DOWNLOADED_FOLDER` |
| Apply a mail-format patch on a clean branch | `bash tools/ftc.sh patch "$HOME/Downloads/change.patch"` |

`FTC_ROBOT_SERIAL` overrides `192.168.43.1:5555`. Explicit serial arguments override
that variable for connect/install/deploy/logs. Every device operation selects a serial;
never let an emulator or another robot become the implicit deployment target. Existing
`ftcgradle` aliases can still be used by people; scripts do not depend on them.

## Working with the physical robot

- A reconnect request authorizes the scripted check/connect/single ADB-server restart.
  The restart affects other ADB clients on this laptop; `FTC_ADB_RESTART=0` disables it.
  Do not kill emulators, Java or Android Studio automatically. Do not repeatedly retry
  unreachable networks or disable host security settings.
- Build/test requests alone do not request installation. Install only for a requested
  deploy/install workflow, after the operator has stopped the OpMode. Do not repeat a
  confirmation already supplied in the session. The scripts do not detect the STOP state.
- Install restarts the RC app. Have the operator verify the expected build on Driver Hub
  INIT afterward. Scripts never press INIT/START or drive/feed/shoot. Physical tests are
  run by the operator. Reconnection is not proof the app or its hardware is ready.
- Download after STOP so traces are closed. Preserve raw data; no calibration resets or
  automatic promotion of saved settings. A missing hive directory is a download failure,
  not an empty successful experiment.
- Remote deletion has exactly one form: `bash tools/ftc.sh prune DOWNLOADED_FOLDER --delete`,
  after STOP, and only when a mentor asks for it in this session. Run it without `--delete`
  first and show the list. It removes only recordings whose checksum equals the downloaded
  copy; never delete hub files any other way. See [recordings and storage](docs/POLLEN_LOGGING.md).
- Do not uninstall the RC app automatically to fix a signing mismatch. Explain the
  failure and have the mentor decide how to resolve it.

## Editing and verification

- Team code belongs in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` and tests
  in `TeamCode/src/test/`. Keep changes small enough for students to understand.
- Preserve `lox/`, existing hardware alternatives and commented-out code unless asked.
  Do not edit upstream `FtcRobotController/`, `build.common.gradle` or
  `build.dependencies.gradle`; team build changes belong in `TeamCode/build.gradle`.
- Treat launcher speeds, PIDF gains, motor directions and calibration as measured data.
  Explicitly call out changes. Preserve the `PollenFacing` route/robot transform boundary.
- Bump `PollenConfig.BUILD` when robot behavior changes. Tool/documentation-only changes
  do not need a new robot build label. Update relevant operator docs and rebuild printable
  drive-team PDFs with `python3 tools/build-drive-team-pdfs.py` when those guides change.
- Run `bash tools/ftc.sh test` for Pollen changes and `git diff --check HEAD` before
  delivery. Add new plain Java tests to `PollenTests` and `test-pollen-sequence.sh`.
  For desktop-tool changes run `python3 tools/test-ftc-tools.py` and
  `python3 tools/test-hive-download.py`; these use fakes, not a connected robot.
- Use Gradle unit tests/build to check Android integration where the SDK is available.
  Report computer tests, Android build, install and physical tests separately. A passing
  simulated route or shell test is not field validation.
- Text files use LF per `.gitattributes` (Windows `.bat`/`.cmd` excepted).

## Shared research notes

- [docs/research/](docs/research/README.md) holds research, options and decisions that are
  not yet rules or tested behavior, so sessions and agents can build on each other's work.
  Read the relevant note before proposing tooling or workflow changes.
- Add findings there with the evidence labels its README defines, say whether you ran in a
  cloud or local session, and add to other contributors' entries rather than rewriting them.
- A note is not permission. The rules in this file apply until a mentor changes this file.

## Git and handoffs

- Never discard, silently stash, reset or clean the user's work. Inspect conflicts and
  preserve local changes. Do not change global Git identity or the default CLI account.
- Desktop sessions: leave verified edits in the working tree and report what changed.
  Commit locally when the user asks, with a clear message on the current branch. Push,
  pull or open PRs only on request, using the existing configured `origin`. Personal
  SSH/key/CLI-account instructions belong outside the team repo. Do not embed keys/tokens.
- Do not use a GitHub connector/API to push, commit, create branches, comment or change
  PRs; do Git work in the local checkout. Do not add `Co-Authored-By` trailers.
- Web/cloud sessions without laptop access (or when a patch is requested): share
  `git format-patch` mail patches named `description-YYYYMMDD-HHMM.patch` (Pacific time),
  state the base SHA, and verify `git am` in a disposable checkout. On the laptop,
  `tools/ftc.sh patch` applies one with a backup branch via `git am`; it does not run
  tests, install or push. Run the tests yourself after applying.
- If two desktop agents are active, use separate worktrees for code edits, or one writer
  at a time in this checkout. Only one session should control deployment/robot testing.
