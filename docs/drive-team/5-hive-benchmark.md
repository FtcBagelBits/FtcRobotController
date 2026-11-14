# Hive pipeline benchmark

**Build pollen-hive-v5. Camera only: no motors or odometry.** Full instructions and export commands:
`docs/POLLEN_HIVE_BENCHMARK.md`. Print this sheet for practice.

## First prove logging works

Select **TEST Hive pipeline benchmark**. INIT: saved pipeline only, **Passes 1 / Seconds 5**,
correct cell ID. START waits. Settle our cell UP, tap **LEFT**; at WAIT settle it DOWN, tap **RIGHT**.
At DONE, STOP and download. Confirm a raw JSONL file and summary exist before longer trials.
Calibration need not have passed; verify the saved lens height is your measured height.

## Prepare profiles once on the Mac

Export originals, copy baseline to spare slots, change one setting per copy. Keep **36h11 / 82.55 mm /
Full 3-D / Single grouping**, identical ID filters and crop. Use Camera source and allow software
switching. Save and export tested profiles, then reconnect camera to Hub. No edits during a run.

| Slot | Suggested variant |
| --- | --- |
| 0 | Working baseline |
| 1 | Fixed exposure about half baseline |
| 2 | Fixed exposure about twice baseline |
| 3 | Lower detector downscale, e.g. 2 to 1 |
| 4 | Higher capture resolution, same downscale |

Use supported values. Already at a limit? Omit that variant or document a gain change instead.
Save the exports: pipeline numbers alone do not identify the settings.

## Full run: four confirmations, about 7 minutes 20 seconds

INIT D-pad up/down selects, left/right edits. Set **First 0 / Last 4 / Passes 2 / Seconds 20**,
correct near cell and scenario. Rear camera faces hive from the actual shooting position.

| Near cell | First ID |
| --- | --- |
| Primary RED / Primary BLUE | 34 / 42 |
| Secondary RED / Secondary BLUE | 30 / 38 |

START waits for you. **Only move the hive at WAIT.** Let it stop rocking before each press.

1. Our near cell **UP**: **LEFT** once; hold still while all profiles run.
2. WAIT: same cell **DOWN**: **RIGHT** once; hold still.
3. WAIT: **UP / LEFT**; hold still. Second pass reverses profile order.
4. WAIT: **DOWN / RIGHT**; hold still. At DONE press **STOP**.

Each profile gets 2 seconds settle + 20 seconds capture. **Circle cancels a mislabeled run**;
STOP/restart. ERROR: stop and fix it. No counting hits or copying telemetry.

## Download and choose what to test next

Mac on Control Hub Wi-Fi, from repo root (new folder name each time):

`bash tools/hive-benchmark-download.sh "$HOME/Desktop/hive-session-01"`

Send the **ZIP plus tested pipeline exports**. Compare UP usable FPS, gaps and ready-time;
any UP during settled DOWN needs investigation. Test winners at the secondary start and small
placement offsets in separate fixed runs. Finish calibration and tipping checks before LIVE shots.

**Limits:** raw logs work without valid tilt; readiness does not. Static trials do not certify
transition timing. The test restores the saved pipeline and never promotes a winner.
