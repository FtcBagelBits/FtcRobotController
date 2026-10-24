import org.firstinspires.ftc.teamcode.pollen.*;
import static org.firstinspires.ftc.teamcode.pollen.PollenShootTuning.Field.*;

/** Mat arithmetic and supporting-shot timing/failure regressions; no robot required. */
public final class PollenMatSupportTest {
    private static int checks;
    private static void check(boolean ok, String message) {
        checks++;
        if (!ok) throw new AssertionError(message);
    }
    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-6, message + ": " + actual + " != " + expected);
    }
    private static void zero(PollenSequence.Output o) {
        check(o.forward == 0 && o.strafe == 0 && o.turn == 0
                && o.launcherVelocity == 0 && o.feederPower == 0, "all outputs zero");
    }
    private static void release(PollenShootTuning t) { t.update(false, false, false, false, false); }
    private static void select(PollenShootTuning t, PollenShootTuning.Field f) {
        while (t.selected() != f) { t.update(false, true, false, false, false); release(t); }
    }
    private static void tap(PollenShootTuning t, int direction) {
        t.update(false, false, direction < 0, direction > 0, false); release(t);
    }
    private static void set(PollenShootTuning t, PollenShootTuning.Field f, int direction, int count) {
        select(t, f);
        for (int i = 0; i < count; i++) tap(t, direction);
    }
    private static PollenShootTuning support() { return new PollenShootTuning(null, true); }
    private static void rejects(Runnable fn) {
        boolean rejected = false;
        try { fn.run(); } catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "invalid move rejected");
    }

    /** Tick every 20 ms from {@code from} up to (not including) {@code to}; returns the last output. */
    private static PollenSequence.Output run(PollenSequence s, double from, double to, double velocity) {
        PollenSequence.Output out = null;
        for (double now = from; now < to - 1e-9; now += .02) out = s.tick(now, velocity, false);
        return out;
    }
    public static void main(String[] args) {
        PollenDriveCalibration.Profile mat = PollenDriveCalibration.Profile.MAT_TRIAL;
        near(PollenDriveCalibration.FORWARD_MAT_COEFFICIENT, 138.17142857142858, "all .25-power trials fit");
        near(PollenDriveCalibration.forwardDistanceCm(1.5, .5), 113, "half-power measurement");
        near(PollenDriveCalibration.forwardDistanceCm(2, .2), 55.26857142857143, "A estimate only");
        near(PollenConfig.PARK_FORWARD_SECONDS, 2, "A tested time remains");
        near(PollenConfig.PARK_FORWARD_POWER, .2, "A tested power remains");
        // Replay observed endpoints: a corrected request for the observed distance
        // should reconstruct the old command's duration and angle at each data knot.
        PollenDriveCalibration.Move m = PollenDriveCalibration.move(mat, 107, .2, 0);
        near(m.seconds, 100 / (129 * .2), "107-cm endpoint recreates old 100-cm forward command");
        near(m.commandAngleDegrees, 0, "forward command");
        m = PollenDriveCalibration.move(mat, 80, .2, 90);
        near(m.seconds, 100 / (129 * .2), "80-cm endpoint recreates old 100-cm strafe command");
        near(m.commandAngleDegrees, 90, "strafe command");
        m = PollenDriveCalibration.move(mat, Math.hypot(54, 31), .2, Math.toDegrees(Math.atan2(31, 54)));
        near(m.seconds, 75 / (129 * .2), "measured diagonal recreates old 75-cm duration");
        near(m.commandAngleDegrees, 45, "measured diagonal recreates old command angle");
        m = PollenDriveCalibration.move(mat, 75, .2, 45);
        check(m.commandAngleDegrees > 56 && m.commandAngleDegrees < 57, "desired 45 needs more sideways command");
        check(m.seconds > 3.5 && m.seconds < 3.6, "desired 75 needs longer drive");
        for (int angle = -90; angle <= 90; angle++) {
            m = PollenDriveCalibration.move(mat, 75, .5, angle);
            check(Double.isFinite(m.seconds) && m.seconds > 0, "finite angular interpolation");
            PollenDriveCalibration.Move mirror = PollenDriveCalibration.move(mat, 75, .5, -angle);
            near(m.seconds, mirror.seconds, "mirror time");
            near(m.strafe, -mirror.strafe, "mirror strafe assumption");
            near(m.forward, mirror.forward, "mirror forward assumption");
            // Explicit command sum shows no mixer saturation at supported powers.
            check(Math.abs(m.forward) + Math.abs(m.strafe) < 1, "no wheel clipping");
        }
        rejects(() -> PollenDriveCalibration.move(mat, Double.NaN, .2, 0));
        rejects(() -> PollenDriveCalibration.move(mat, 50, 0, 0));
        rejects(() -> PollenDriveCalibration.move(mat, 50, .2, 91));

        // CAD-derived nominal routes: 18-inch footprint, no chassis rotation.
        // Primary: forward 10 cm off the wall, left 25 in, then to (-56,-26).
        double primaryForward = 70.585 - 9 - 10 / 2.54 + 26, primaryLeft = -12.74164098 + 56 - 25;
        near(PollenFieldGeometry.primaryDistanceCm(), Math.hypot(primaryForward, primaryLeft) * 2.54, "primary third leg");
        near(PollenFieldGeometry.primaryAngleDegrees(), Math.toDegrees(Math.atan2(primaryLeft, primaryForward)), "primary third-leg angle");
        check(Math.abs(PollenFieldGeometry.primaryDistanceCm() - 217.47) < .01, "primary third leg about 217.47 cm");
        check(Math.abs(PollenFieldGeometry.primaryAngleDegrees() - 12.31) < .01, "primary third leg about +12.31 degrees");
        // Support: flush/square, centered between flower edge and midline; 8 in forward, then to (-56,-47).
        near(PollenFieldGeometry.SUPPORT_START_X_INCHES, -10.029, "support center splits the gap");
        near(PollenFieldGeometry.supportFlowerGapCm(), PollenFieldGeometry.supportMidlineGapCm(), "equal flower and midline gaps");
        near(PollenFieldGeometry.supportFlowerGapCm(), 1.029 * 2.54, "each side gap");
        near(PollenFieldGeometry.supportOffsetFromHiveCm(), (12.74164098 - 10.029) * 2.54, "offset toward midline from hive center");
        near(PollenFieldGeometry.supportStartZInches(), -61.585, "flush start keeps the centered shooting distance");
        double supportRight = 56 - 10.029, supportForward = -47 + 61.585 - 8;
        near(PollenFieldGeometry.supportDistanceCm(), Math.hypot(supportRight, supportForward) * 2.54, "support diagonal distance");
        near(PollenFieldGeometry.supportAngleDegrees(), -Math.toDegrees(Math.atan2(supportRight, supportForward)), "support right-forward angle");
        near(PollenFieldGeometry.PRIMARY_PARK_Z_INCHES - PollenFieldGeometry.SUPPORT_PARK_Z_INCHES - 18,
                3, "two nominal parking footprints have a three-inch gap");
        PollenShootTuning t = support();
        near(t.value(START_DELAY), 15, "support waits 15 seconds");
        near(t.value(FEED_SECONDS), 7.5, "support feed duration");
        near(t.value(LEG3_CM), PollenFieldGeometry.supportDistanceCm(), "support field distance");
        check(t.forward() > 0 && t.strafe() > 0, "support moves right AND forward");
        check(t.profile() == mat, "new experimental support uses trial profile");
        check(t.worstCaseSeconds() > 25.7 && t.worstCaseSeconds() < 26, "overlapped spinup fits field route");
        near(t.value(LEG1_CM), 20.32, "forward clearance before rightward support leg");
        near(t.value(LEG1_ANGLE), 0, "support leg 1 straight forward");
        near(t.value(LEG2_CM), 0, "support leg 2 unused by default");
        near(t.value(LEG3_ANGLE), PollenFieldGeometry.supportAngleDegrees(), "support leg 3 angle");
        PollenSequence s = t.sequence(); s.start(0);
        zero(s.tick(0, 0, false)); zero(s.tick(10.99, 0, false));
        PollenSequence.Output o = s.tick(11, 0, false);
        near(o.launcherVelocity, 1250, "spinup at t=11"); near(o.feederPower, 0, "no early feed");
        s.tick(14.7, 1250, false);
        o = s.tick(14.99, 1250, false); near(o.feederPower, 0, "ready still cannot feed before t=15");
        o = s.tick(15, 1250, false); near(o.feederPower, 1, "feed at t=15 after qualifying speed");
        near(o.forward, 0, "stationary shot"); near(o.strafe, 0, "stationary shot strafe");
        near(s.tick(22.49, 900, false).feederPower, 1, "full 7.5 seconds despite normal dip");
        zero(s.tick(22.5, 1250, false));
        o = s.tick(22.51, 0, false); near(o.forward, .45, "support forward clearance first");
        near(o.strafe, 0, "no strafe beside the flower");
        double leg1Ends = 22.51 + t.legMove(1).seconds;  // clock starts on the first commanding loop
        check(run(s, 22.53, leg1Ends - .005, 0).forward == .45, "leg 1 runs its full time");
        o = s.tick(leg1Ends + .001, 0, false); zero(o); check(o.phase.startsWith("NEXT"), "leg 1 ends on time");
        double leg3Start = leg1Ends + .021;
        o = s.tick(leg3Start, 0, false); near(o.forward, t.forward(), "support second leg forward component");
        near(o.strafe, t.strafe(), "rightward departure"); near(o.launcherVelocity, 0, "shooter off before drive");
        near(o.feederPower, 0, "feeding off before drive");
        check(run(s, leg3Start + .02, leg3Start + t.driveSeconds() - .005, 0).strafe == t.strafe(), "leg 3 full time");
        zero(s.tick(leg3Start + t.driveSeconds() + .001, 0, false));
        o = run(s, leg3Start + t.driveSeconds() + .021, leg3Start + t.driveSeconds() + .55, 0);
        check(o.finished && o.phase.startsWith("DONE"), "finish before deadline"); zero(o);
        check(leg3Start + t.driveSeconds() + .55 < 26.5, "real loop transitions still finish before the 27 s cutoff");

        // Spinup timeout preserves the partner's entire scheduled shooting slot.
        s = t.sequence(); s.start(0); s.tick(11, 0, false);
        zero(s.tick(15, 0, false)); check(s.warning().contains("retaining"), "skipped shot warning");
        zero(s.tick(22.49, 0, false)); zero(s.tick(22.5, 0, false));
        near(s.tick(22.51, 0, false).forward, .45, "failed spinup begins same forward clearance, not early");
        s = t.sequence(false); s.start(0); zero(s.tick(14.99, 0, false));
        zero(s.tick(15, 0, false)); zero(s.tick(22.49, 0, false)); zero(s.tick(22.5, 0, false));
        near(s.tick(22.51, 0, false).forward, .45, "INIT failure preserves slot and forward clearance");

        for (double stopAt : new double[] {1, 11.1, 15.1, 23, 25.5}) {
            s = t.sequence(); s.start(0);
            for (double now = 0; now < stopAt; now += .02) s.tick(now, 1250, false);
            zero(s.tick(stopAt, 1250, true)); o = s.tick(stopAt + 1, 1250, false);
            zero(o); check(o.finished, "STOP latches through support phases");
        }
        s = t.sequence(); s.start(0); o = s.tick(11, Double.NaN, false); zero(o); check(o.finished, "bad velocity during prespin stops");
        s = t.sequence(); s.start(0); o = s.tick(27, 1250, false); zero(o); check(o.finished, "deadline stops support");
        // Operator-requested feed=0 removes prespin and the feed slot.
        PollenShootTuning skip = support(); set(skip, FEED_SECONDS, -1, 15);
        s = skip.sequence(); s.start(0); zero(s.tick(11, 0, false));
        near(s.tick(15, 0, false).forward, .45, "explicit feed=0 departs at selected delay");
        set(skip, LEG3_CM, -1, 80); set(skip, LEG1_CM, -1, 30); s = skip.sequence(); s.start(0); zero(s.tick(15, 0, false));
        check(s.tick(15.5, 0, false).finished, "zero feed/distance holds only");

        set(t, LEG3_CM, 1, 80); check(t.worstCaseSeconds() < 26, "support rejects over-budget distance");
        check(t.notice().contains("rejected"), "budget warning");
        set(t, DRIVE_POWER, -1, 1); near(t.value(DRIVE_POWER), .45, "reject slower over-budget power");
        t.sequence();
        PollenShootTuning remembered = new PollenShootTuning(t.snapshot(), true);
        near(remembered.value(LEG3_CM), t.value(LEG3_CM), "support memory");
        check(remembered.profile() == mat, "remember calibration profile");
        select(t, CALIBRATION); t.lock(); tap(t, -1); check(t.profile() == mat, "lock calibration at START");
        PollenShootTuning shortDelay = support(); set(shortDelay, START_DELAY, -1, 40);
        near(shortDelay.value(START_DELAY), 4, "feeding needs four-second prespin window");
        // Reviewer's case: a calibrated C run at 4 s delay, power 0.25, leg 3 lengthened by 10 cm.
        set(shortDelay, DRIVE_POWER, -1, 4); near(shortDelay.value(DRIVE_POWER), .25, "calibration power");
        set(shortDelay, LEG3_CM, 1, 2);
        double calibratedLeg3 = shortDelay.value(LEG3_CM);
        check(Math.abs(calibratedLeg3 - (PollenFieldGeometry.supportDistanceCm() + 10)) < .01, "calibrated leg 3 (edits round to 0.01 cm)");
        check(shortDelay.budgetOk(), "valid at 4 s delay");
        remembered = new PollenShootTuning(shortDelay.snapshot(), true);
        near(remembered.value(START_DELAY), 15, "delay resets to support default");
        near(remembered.value(DRIVE_POWER), .25, "calibrated power kept, not raised to 0.45");
        near(remembered.value(LEG3_CM), calibratedLeg3, "calibrated leg 3 kept");
        check(remembered.profile() == mat, "calibration profile kept");
        check(!remembered.budgetOk(), "over budget after the delay reset");
        check(remembered.notice().contains("OVER TIME BUDGET") && remembered.notice().contains("Will not start"),
                "over-budget notice says why and that START will not move: " + remembered.notice());
        boolean refused = false;
        try { remembered.sequence(); } catch (IllegalArgumentException expected) { refused = true; }
        check(refused, "an over-budget route cannot be built, so START cannot move the robot");
        double worst = remembered.worstCaseSeconds();
        set(remembered, LEG3_CM, 1, 1);
        near(remembered.value(LEG3_CM), calibratedLeg3, "edits that lengthen an over-budget route are refused");
        check(remembered.notice().contains("longer"), "refusal explained");
        set(remembered, FLYWHEEL, 1, 1);
        near(remembered.value(FLYWHEEL), 1300, "edits that do not change the time are allowed");
        set(remembered, START_DELAY, -1, 1);
        check(remembered.worstCaseSeconds() < worst, "lowering delay is allowed while over budget");
        while (!remembered.budgetOk()) set(remembered, START_DELAY, -1, 1);
        check(remembered.value(START_DELAY) >= 4 && remembered.budgetOk(), "operator can work back under budget");
        near(remembered.value(LEG3_CM), calibratedLeg3, "route still the calibrated one");
        near(remembered.value(DRIVE_POWER), .25, "power still the calibrated one");
        remembered.sequence();
        shortDelay.update(false, false, false, false, true);
        near(shortDelay.value(START_DELAY), 15, "triangle support wait");
        near(shortDelay.value(FEED_SECONDS), 7.5, "triangle support feed");
        near(shortDelay.value(LEG3_CM), PollenFieldGeometry.supportDistanceCm(), "triangle support field distance");
        check(new PollenShootTuning(null).value(FEED_SECONDS) == 10, "B defaults independent of support");

        // Square gives B a three-leg route without changing triangle's tested default.
        PollenShootTuning primary = new PollenShootTuning(null);
        primary.update(false, false, false, false, false, true); release(primary);
        near(primary.value(LEG1_CM), 10, "field preset first leaves the wall by 10 cm");
        near(primary.value(LEG1_ANGLE), 0, "wall clearance is straight forward");
        near(primary.value(LEG2_CM), 63.5, "then clears the frame to the left");
        near(primary.value(LEG2_ANGLE), 90, "hive clearance is pure left");
        near(primary.value(LEG3_CM), PollenFieldGeometry.primaryDistanceCm(), "then travel to near parking slot");
        near(primary.value(LEG3_ANGLE), PollenFieldGeometry.primaryAngleDegrees(), "third-leg angle");
        near(primary.value(DRIVE_POWER), .25, "primary field trial power");
        check(primary.worstCaseSeconds() < 26, "field preset fits full shot budget");
        double legs = primary.legMove(1).seconds + primary.legMove(2).seconds + primary.legMove(3).seconds;
        near(primary.worstCaseSeconds(), 4 + 10 + .5 + legs, "budget counts all three legs");
        s = primary.sequence(false); s.start(0); o = s.tick(0, 0, false);
        near(o.forward, .25, "leg 1 forward first"); near(o.strafe, 0, "leg 1 has no strafe beside the wall");
        double t1 = primary.legMove(1).seconds;
        run(s, .02, t1 - .005, 0);
        zero(s.tick(t1 + .001, 0, false));
        double t2start = t1 + .021, t2 = t2start + primary.legMove(2).seconds;
        o = s.tick(t2start, 0, false);
        near(o.forward, 0, "leg 2 left only"); near(o.strafe, -.25, "leg 2 left");
        run(s, t2start + .02, t2 - .005, 0);
        zero(s.tick(t2 + .001, 0, false));
        o = s.tick(t2 + .021, 0, false);
        near(o.forward, primary.forward(), "third leg after clearances"); near(o.strafe, primary.strafe(), "third leg angle");
        set(primary, LEG3_CM, -1, 100); near(primary.value(LEG3_CM), 0, "zero main leg");
        check(primary.legMove(2).seconds > 0, "clearance remains independently enabled");
        s = primary.sequence(false); s.start(0); near(s.tick(0, 0, false).forward, .25, "zero main leg still performs leg 1");
        set(primary, LEG1_CM, -1, 20); near(primary.value(LEG1_CM), 0, "leg 1 can be skipped");
        s = primary.sequence(false); s.start(0); near(s.tick(0, 0, false).strafe, -.25, "skipped leg 1 starts with leg 2");
        primary.update(false, false, false, false, true); release(primary);
        near(primary.value(LEG1_CM), 0, "triangle removes wall leg");
        near(primary.value(LEG2_CM), 0, "triangle removes dogleg");
        near(primary.value(LEG3_ANGLE), 25.87, "triangle baseline angle");
        near(primary.driveSeconds(), 2.3627906976744186, "triangle original duration");
        select(primary, CALIBRATION); tap(primary, 1);
        remembered = new PollenShootTuning(primary.snapshot()); check(remembered.profile() == mat, "B profile memory");
        primary.lock(); primary.update(false, false, false, false, false, true);
        near(primary.value(LEG2_CM), 0, "START locks field-preset button");

        // Every leg distance/angle and the launcher speed are controller rows that reach the sequence.
        PollenShootTuning tune = new PollenShootTuning(null);
        tune.update(false, false, false, false, false, true); release(tune);
        set(tune, LEG1_CM, 1, 3); near(tune.value(LEG1_CM), 13, "leg 1 cm in 1-cm steps");
        set(tune, LEG1_ANGLE, -1, 2); near(tune.value(LEG1_ANGLE), -2, "leg 1 angle editable");
        set(tune, LEG2_CM, -1, 1); near(tune.value(LEG2_CM), 58.5, "leg 2 cm in 5-cm steps");
        set(tune, LEG2_ANGLE, -1, 3); near(tune.value(LEG2_ANGLE), 87, "leg 2 angle editable");
        double fractional = tune.value(LEG3_ANGLE);
        set(tune, LEG3_ANGLE, 1, 1); near(tune.value(LEG3_ANGLE), Math.floor(fractional) + 1, "leg 3 angle snaps to whole degree");
        set(tune, FLYWHEEL, 1, 4); near(tune.value(FLYWHEEL), 1450, "launcher speed editable");
        for (int leg = 1; leg <= 3; leg++) {
            PollenDriveCalibration.Move expect = PollenDriveCalibration.move(mat, tune.legCm(leg), .25, tune.legAngle(leg));
            near(tune.legMove(leg).seconds, expect.seconds, "leg " + leg + " time uses its own edited values");
            near(tune.legMove(leg).strafe, expect.strafe, "leg " + leg + " direction uses its edited angle");
        }
        s = tune.sequence(); s.start(0);
        near(s.tick(0, 0, false).launcherVelocity, 1450, "edited launcher target reaches sequence");
        check(tune.legMove(1).strafe > 0, "negative leg 1 angle drifts right");
        for (PollenShootTuning.Field f : new PollenShootTuning.Field[] {LEG1_ANGLE, LEG2_ANGLE, LEG3_ANGLE}) {
            set(tune, f, 1, 200); near(tune.value(f), 90, "angle upper limit");
            set(tune, f, -1, 200); near(tune.value(f), -90, "angle lower limit");
        }
        set(tune, LEG1_CM, 1, 100);
        check(tune.worstCaseSeconds() < 26, "long field-route edits stay inside budget");
        check(tune.value(LEG1_CM) < 60 && tune.notice().contains("rejected"), "over-budget leg edit rejected visibly");
        PollenShootTuning shortRoute = new PollenShootTuning(null);
        set(shortRoute, LEG1_CM, 1, 100); near(shortRoute.value(LEG1_CM), 60, "leg 1 upper limit");
        PollenShootTuning kept = new PollenShootTuning(tune.snapshot());
        near(kept.value(LEG2_ANGLE), tune.value(LEG2_ANGLE), "leg edits survive next INIT");
        near(kept.value(FLYWHEEL), 1450, "launcher edit survives next INIT");
        PollenShootTuning cTune = support();
        set(cTune, LEG2_CM, 1, 1); near(cTune.value(LEG2_CM), 5, "C optional leg 2 can be enabled");
        near(cTune.value(LEG2_ANGLE), -90, "C leg 2 defaults to robot-right");
        set(cTune, FLYWHEEL, -1, 2); near(cTune.value(FLYWHEEL), 1150, "C launcher speed editable");
        s = cTune.sequence(); s.start(0); s.tick(10.99, 0, false);
        near(s.tick(11, 0, false).launcherVelocity, 1150, "C prespin uses edited speed");
        System.out.println("PollenMatSupportTest: " + checks + " checks passed");
    }
}
