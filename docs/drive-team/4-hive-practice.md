# Hive calibration and practice

**Build pollen-hive-v3 - experimental.** Print front/back. Full steps: `docs/POLLEN_HIVE_TESTING.md`.
Keep existing match autos available. Rear camera/launcher face hive; **front intake faces end wall**.

<img src="img/hive-height.svg" style="width:4.3in">

## Camera setup and calibration (NO MOTORS)

Limelight named **limelight**; USB to Hub blue port. Pipeline 0: 36h11, **82.55 mm**, full 3-D.
Use **TEST Hive camera (NO MOTORS)**. It maps no motors or odometry and runs until STOP.

| Step | Do this |
| --- | --- |
| 1. Measure in INIT | Floor to lens center (default **14 in**); floor to UP/DOWN tag-row centers (**49.7 / 35.6 in** nominal). D-pad selects/edits camera rows. Triangle resets one row; Square saves. |
| 2. Fit tilt | PRIMARY start, square to wall, centered. Near cell **physically UP**. START. Check correct IDs; see all four for sideways fit. Square, hands off for **3 s**. Display uses CANDIDATE. |
| 3. Check UP | Still UP: D-pad **left**, then Circle. Hold still 3 s. Expect **PASS**. |
| 4. Check DOWN | Tip same cell DOWN, let it settle. D-pad **right**, Circle. Expect PASS, or **BLOCKED ONLY** if tags hidden. Hidden is not a DOWN height measurement. |
| 5. Save/reload | Cross saves after both checks. STOP / INIT / START verifies reload. Save before moving. Triangle forgets the cell/checks after changing starts. |

**Cell IDs:** primary red 34-37, blue 42-45; secondary red 30-33, blue 38-41. Confirm on the real hive.
Square starts a fresh candidate and clears checks. A rejected fit cannot save an old candidate.
Wrong label / noisy readings / inconsistent tags = reject. Correct measurements in INIT and repeat;
**no automatic DOWN-height adjustment**. Cross saves exactly the previewed numbers.

## Validate at each shooting start

- With the near cell UP, identify it, then check UP and DOWN. At secondary starts, validate the
  same mount; do not refit sideways from the offset position.
- Leave the display running: tip **20 times**, cover our tags while the other hive is visible,
  unplug/replug, rock through the middle. **Zero false UPs**. UNKNOWN means no shot permission.
- Test small placement errors, then restore S. A rigid mount, level chassis and negligible camera
  roll are assumed. Recalibrate if mount/height/pipeline geometry changes.

**Record:** lens _____ in; UP / DOWN tags _____ / _____ in; tilt _____ deg; cell IDs _____;
false UP _____ / 20. Capture reports save automatically. The screen tests the **camera gate only**.

<div class="page-break"></div>

## Practice one behavior at a time

| Driver Hub mode | Default / what happens |
| --- | --- |
| TEST Hive primary shot ONLY | **DRY**. No drive in any setting. LIVE fires one volley or stops at first-shot window. |
| TEST Hive secondary shot ONLY | **DRY**. No drive. LIVE waits for UP, fires, stops. |
| TEST Hive garden + return | **DRIVE**. No launcher/agitator. LIVE adds collection intake. Returns to S and stops, no park. |
| EXPERIMENTAL Hive Primary + Garden | **LIVE**. Shoot, coast, collect, return, wait/shoot if UP, park. |
| EXPERIMENTAL Hive Secondary | **LIVE**. Wait for UP, shoot, park. |

**Order:** camera validation; shot ONLY DRY then LIVE with a small batch; garden DRIVE without
balls then LIVE pickup; full autos last. Someone stays on STOP. Every motor run requires Pinpoint
READY and ends by **27 s**. DRY uses virtual launcher speed; actual LIVE shots require measured speed,
fresh steady UP and a settled pose. Camera-only mode is the indefinite test, not DRY full auto.

## Motor-mode INIT menu

D-pad row/value; LB/RB jump five. **Triangle** resets one row. **Circle** resets route values only,
keeping camera and Execution. **Square** saves. Execution is never saved and resets each INIT to
the mode's default above. Saved practice settings are shared with the corresponding full auto.
Check BUILD, run type, Execution, CHANGED values, camera profile, cell/alliance and Pinpoint READY.

<img src="../POLLEN_HIVE_GARDEN.svg" style="width:4.9in;max-height:2.7in">

Start every garden test at **S**. Heading changes aim the **front intake** toward the garden;
return restores rear-to-hive shooting. No midrun odometry reset. No turn at E: body clearance is
only **1 in**. Four ball centers need about **17.45 in** of centered intake width; measure actual reach.

**After practice:** pull `/sdcard/FIRST/pollen-hive` with adb (settings + reports), and the Robot
Controller log. Reports record checks, settings, phases, elapsed time and pose. Write down pickup
count, return error, cycle time and false-UP count. Use `tools/hive-promote-defaults.py` to preview
proven changes; `--write` promotes them. Test/build/install once after tuning, not after every run.
