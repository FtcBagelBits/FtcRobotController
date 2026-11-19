# How PollenBot drives, and what to tune on each floor

2026-11-18, Codex cloud session. Source review of `pollen-collection-practice` at `145b2d8`, build `pollen-hive-v19`. This session had no physical robot.

**[inferred] Recommendation.** Tune garden pickup and flower draining with the current controller. Trial Pedro's full follower on the longer trip from the shooting mark to the garden entry and back. Use vinyl to debug the code and the geometry. Reserve tile time for traction, stopping accuracy, and actual ball counts.

## Follow one garden pass from target to wheels

1. **[repo] Establish the target.** The default garden start is square to the wall with the front chassis edge 15.24 cm away. The requested final clearance is 2.54 cm, so the drive target is **12.70 cm forward from the observed entry pose**. The sequence holds the entry heading. It can start inside a larger autonomous route without resetting Pinpoint. [Collection sequence][collection]
2. **[repo] Measure progress.** The two odometry pods track forward and lateral travel. Pinpoint supplies a pose through Pedro's `PinpointLocalizer`. Each loop reads position and heading. `PollenFacing` converts between the team's route coordinates and the physical chassis orientation so the intake goes in first. [Odometry adapter][odo] · [Facing conventions][facing]
3. **[repo] Command the wheels.** The team's controller computes the distance and direction to the target, accounts for current heading, and mixes forward, sideways, and turning commands for the four mecanum wheels. The 0.20 approach setting caps translational power. Power ramps up, and measured velocity helps the controller anticipate coasting as it approaches the target. [Drive controller][drive] · [Motion controller][motion]
4. **[repo] Decide whether the pass is over.** A normal hive or collection move advances within 0.5 in (1.27 cm) and 2° of its target, with translation at or below 1.5 in/s and rotation at or below 10°/s. The controller watches for a stalled loop, lack of progress, and a move timeout. The garden routine then dwells, backs off, and re-enters according to its INIT settings. The emergency timed autos use elapsed time and commanded power to determine travel. [Motion controller][motion] · [Autonomous families][reference]

**[inferred] What the tape tells you.** If you place the robot 5 cm farther from the wall, an accurate 12.70 cm drive still leaves it about 5 cm too far away. Mark a front edge and a side edge so the starting distance and angle repeat. Compare Pinpoint's reading with those marks to distinguish a placement error from a drive error.

### Flower contact and the sideways swings

**[repo]** At START the intake runs at full inward power and the arm lowers. From the default 20 cm stand-off, the robot approaches 17 cm by odometry, then pushes at 0.15 power for 0.30 s. It uses the pose actually reached as the center for the swings. A cycle strafes to 3 cm left of center and then 3 cm right of center. The heading target stays fixed, with steering corrections for drift. After three cycles the robot centers, backs out 18 cm with the drainer down, raises it, and makes two 7 cm forward-and-back intake passes. [Collection sequence][collection] · [Setup and INIT settings][practice]

**[inferred]** A 1.27 cm arrival tolerance is large compared with a 3 cm strafe target. Put a chassis pointer over tape marks at center and ±3 cm, run one cycle, and measure both extremes. Pinpoint estimates chassis travel. Seating, arm flex, loose balls, and balls retained need a physical observation.

## What Pedro would change

**[repo]** PollenBot already uses Pedro's localizer. The Odo, Hive, and collection autos use team code to control the drive. The repo includes Pedro 3, a mecanum configuration whose motor names and directions match PollenBot, and a separate **Pedro Test** OpMode that uses the full follower. The checked-in Foresight coefficients are examples from another robot. The tuning dependency is present, but the tuner registration and procedures are missing. [Team code reference][reference] · [Pedro configuration][constants]

| Step | Work and evidence |
| --- | --- |
| Verify the pose | [repo] Pod directions and measured distances were checked after swapping the pod cables. The configured offsets are still Lox's **156 and −81 mm**. A vinyl spin suggested **167.8 and −67.6 mm**. Repeat turns on tiles in both directions and measure how far the reported center wanders before adopting offsets. [Calibration record][reference] · [Baseline procedure][baseline] |
| Tune the follower | [doc] Register the Pedro 3 tuning procedures, deploy, and use AutoTune on the robot's network. Verify drivetrain and localization first. Foresight measures forward and strafe speeds, deceleration, braking, and feedback gains. Save the generated configuration after validating it on PollenBot. [Tuning order][tuning] · [Pinpoint procedure][pinpoint] · [Foresight procedure][foresight] |
| Exercise it separately | [repo] In **Pedro Test**, hold Cross and then press and hold Circle for the 24-inch straight path. Circle alone selects the curve. Release Circle to end path following. Both paths preserve heading. These controls follow the executable loop in [Pedro Test][pedro-test]. |
| Try one transit route | [inferred] Compare the current controller and tuned Pedro on shooting mark → garden entry → shooting mark. Translate route coordinates into Pedro's frame once. At handoff, release endpoint hold and give the pickup routine sole drive control. Carry the same pose estimate through both, and preserve the shooter's time budget, STOP and fault handling, intake logic, and hive decisions. [Pedro follower states][states] · [Collection handoff][practice] |

## Which work needs competition tiles?

| Work | Vinyl at home | Competition-style tiles |
| --- | --- | --- |
| Pinpoint baseline | [repo] Tape a 48-inch line. Push forward and sideways by hand, check signs and distance, then check a known 90° turn. [Team baseline][baseline] | [inferred] Repeat the spin-in-place offset check. Pod mounting geometry stays fixed, but slip or a wandering pivot can bias the estimate. |
| Pedro setup and path logic | [inferred] Build, open AutoTune, verify the controls, and try the line and curve with room for overrun. Vinyl measurements can serve for home experiments. | [inferred] Use these tiles for the match follower settings. Especially check strafe braking, overshoot, and return heading. The tuner measures those chassis dynamics directly. [Foresight procedure][foresight] |
| Garden pickup | [inferred] Tape the wall, start, and exit. Try one pass, then three. With loose balls, check whether the intake captures them. | [inferred] Use the garden layout and real pollen to tune final clearance, drive power, and number of passes. Count retained balls and record elapsed time. |
| Flower drain | [inferred] Check sequence order and strafe amplitude. With a flower, also check arm height and insertion alignment. | [inferred] Use a real flower and pollen to tune seating pressure and duration, shake width, and cleanup passes under load. Watch for binding and count retained balls. |

**[inferred] Suggested use of scarce field time.** At home, measure the flower strafe and rehearse the marked garden entry. At practice, run the short Pinpoint spin check first. If field elements are available, prioritize ball retention. If only tiles are available, run Pedro braking and transit trials. A clear patch of competition-style tiles with sufficient travel and coasting room is enough for follower tuning. The hive becomes useful when testing the resulting shooting pose and shot.

**[inferred] Adoption criterion.** Repeat both controllers from the same tape marks. For each run, record transit time, garden-entry position and heading error, and return position and heading error against the physical shooting mark. During pickup trials, also record balls presented/retained and any jam. Adopt Pedro for transit if it saves time consistently while preserving shooting-position repeatability and pickup alignment.

## Decision log

2026-11-18, Codex cloud session — **Proposed** a transit comparison. Field measurements and the team's adoption decision are pending.

[collection]: ../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenCollectionSequence.java
[odo]: ../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenOdometryHardware.java
[facing]: ../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenFacing.java
[drive]: ../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenOdometryDrive.java
[motion]: ../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenHiveMotion.java
[constants]: ../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java
[pedro-test]: ../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/PedroLocalizationTest.java
[reference]: ../POLLEN_CODE_REFERENCE.md
[practice]: ../POLLEN_COLLECTION_PRACTICE.md
[baseline]: ../ODOMETRY_BASELINE_TEST.md
[tuning]: https://pedropathing.com/docs/pathing/tuning
[pinpoint]: https://pedropathing.com/docs/pathing/tuning/localization/pinpoint
[foresight]: https://pedropathing.com/docs/pathing/tuning/foresight
[states]: https://pedropathing.com/docs/pathing/guide/follow-state
