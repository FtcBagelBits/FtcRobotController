# Hive match setup and practice

**Build pollen-hive-v44 - experimental.** Print front/back. Full steps: `docs/POLLEN_HIVE_TESTING.md`.
Keep existing match autos available. Rear camera/launcher face hive; **front intake faces end wall**.

<img src="img/hive-height.svg" style="width:4.3in">

For intermittent tags, run **Calibration Hive Pipeline Benchmark** first (separate benchmark card).

## Camera setup and calibration (NO MOTORS)

Limelight named **limelight**; USB to Hub blue port. Saved AprilTag pipeline (code default 4): 36h11, **82.55 mm**, full 3-D.
Use **Calibration Hive Camera (No Motors)**, in the **TeleOp list**. It maps no motors or odometry and runs until STOP.

| Step | Do this |
| --- | --- |
| 1. Measure in INIT | Floor to lens center (default **11.75 in**); floor to UP/DOWN tag-row centers (**49.7 / 35.6 in** nominal). D-pad selects/edits camera rows. Triangle resets one row; Square saves. |
| 2. Fit tilt | PRIMARY start (1 in from the center seam: launcher on the hive center), square to wall. Near cell **physically UP**. START. Check correct IDs; see all four for sideways fit. Square, hands off for **3 s**. Display uses CANDIDATE. |
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
| Calibration Primary Shot Only | **LIVE**. No drive. Fires within the first-shot window without checking UP. |
| Calibration Secondary Shot Only | **LIVE**. No drive. Waits for UP, fires, stops. |
| Calibration Garden + Return | **LIVE**. No launcher/agitator; collection intake on. Returns to S and stops, no park. |
| Primary Shooter + Garden | **LIVE**. Shoot, one garden approach, return to S, shoot if UP, park. 30 s cutoff. |
| Secondary Shooter + Flower | **LIVE**. Wait for UP, shoot, drain with two swings and one cleanup cycle, return to S, shoot again with no camera wait, park. 30 s cutoff. |
| Beta Primary Shooter - Hero | **LIVE**. Primary garden plan, own saved settings. |
| Beta Primary Shooter - Fast | **LIVE**. Like Hero: closer garden entry, 2.5 s feeds, return nudge. |
| Beta Primary Shooter - Angled Left | **LIVE**. Same start as Primary. Drives straight to a spot 8 in off the wall and 8 in left (toward the garden), turns about 12 deg right to aim at our cell, shoots, drains the garden, comes back to the same spot, aims, shoots, parks. INIT rows **Angled shot spot left in** / **away from wall in** move the spot; the aim angle follows. **Aim target in left of center** (default -2.0 = 2 in right, 0.5 in steps) shifts where it aims: about 18.5 deg right at the defaults (10.5 in left of the start; it aims the launcher). |
| Beta Secondary Shooter - Angled | **LIVE** (not yet run on the robot). Secondary start. Straight off the wall 8 in, right 10.5 in, turns about 4.6 deg left to aim the launcher (odometry), waits for UP, shoots, drains the flower, back to the spot, shoots again with no camera wait, parks OUTER. Own saved profile `secondary-angled`. |
| Beta Secondary Shooter - Out of the Way | **LIVE**. Wait for UP, shoot, wait in the far corner, park on its deadline. |

Validate the camera, then test shot-only and garden (all run LIVE: there is no Execution row) before
running full autos. Keep someone on STOP and check Pinpoint READY. Match autos and Hero pre-spin on
return. **Park margin s** defaults to **1.0 s** (0.5 s steps). Park can interrupt feed.
LIVE feeds from a settled pose at measured launcher speed for up to **3 s**. The primary return shot and the secondary
preload need over **1 s** of confirmed UP; the secondary return shot does not wait for the camera. Match volleys add **3 s** of agitator + launcher (**Clear-out s**).

## Motor-mode INIT menu

**Match and Beta:** On the first INIT row press **left for RED** or **right for BLUE**. Check tag IDs before START.

D-pad row/value; LB/RB jump five. **Triangle** resets one row. **Circle** resets route values only,
keeping camera. **Square** saves. No Execution row: every mode runs LIVE. Rows to check or
change come first (Alliance or side, then delay, launcher, feeds, cycle counts, park margin); tuning,
route points and unused rows follow; camera rows last. Collection and shooting intake default to 1.00.
Check BUILD, run type, CHANGED values, camera profile, cell/alliance and Pinpoint READY.

<div class="side" markdown="1">
<img src="../POLLEN_HIVE_GARDEN.svg">

<div markdown="1">

The diagram shows **Calibration Garden + Return** from **S**. **Calibration Garden Pickup** starts at
the garden entry and finds the right wall, then the wall behind the balls. Four ball centers need
about **17.45 in** of centered intake width. Measure actual reach.

Before departure, a shift over 2 in or 10° stops the run. Check drive wiring if this trips.
After STOP, download the reports with `bash tools/ftc.sh logs NEW_FOLDER`. Record pickup count,
return error, cycle time, and false UPs. See the full guide for promoting saved settings.

</div>
</div>
