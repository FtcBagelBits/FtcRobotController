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
        near(PollenFieldGeometry.primaryDistanceCm(), 227.2483917414033, "primary second leg");
        near(PollenFieldGeometry.primaryAngleDegrees(), 11.775488782504011, "primary second-leg angle");
        near(PollenFieldGeometry.supportDistanceCm(), 115.93474340073678, "support diagonal distance");
        near(PollenFieldGeometry.supportAngleDegrees(), -79.26113999702018, "support right-forward angle");
        near(PollenFieldGeometry.PRIMARY_PARK_Z_INCHES - PollenFieldGeometry.SUPPORT_PARK_Z_INCHES - 18,
                3, "two nominal parking footprints have a three-inch gap");
        PollenShootTuning t = support();
        near(t.value(START_DELAY), 15, "support waits 15 seconds");
        near(t.value(FEED_SECONDS), 7.5, "support feed duration");
        near(t.value(DISTANCE_CM), PollenFieldGeometry.supportDistanceCm(), "support field distance");
        check(t.forward() > 0 && t.strafe() > 0, "support moves right AND forward");
        check(t.profile() == mat, "new experimental support uses trial profile");
        check(t.worstCaseSeconds() > 25.7 && t.worstCaseSeconds() < 26, "overlapped spinup fits field route");
        near(t.value(CLEARANCE_CM), 20.32, "forward clearance before rightward support leg");
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
        double secondLegStart = 22.5 + t.clearanceMove().seconds + .001;
        zero(s.tick(secondLegStart, 0, false));
        o = s.tick(secondLegStart + .01, 0, false); near(o.forward, t.forward(), "support second leg forward component");
        near(o.strafe, t.strafe(), "rightward departure"); near(o.launcherVelocity, 0, "shooter off before drive");
        near(o.feederPower, 0, "feeding off before drive");
        zero(s.tick(secondLegStart + t.driveSeconds() + .001, 0, false));
        o = s.tick(secondLegStart + t.driveSeconds() + .51, 0, false); check(o.finished, "finish before deadline"); zero(o);

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
        set(skip, DISTANCE_CM, -1, 80); set(skip, CLEARANCE_CM, -1, 30); s = skip.sequence(); s.start(0); zero(s.tick(15, 0, false));
        check(s.tick(15.5, 0, false).finished, "zero feed/distance holds only");

        set(t, DISTANCE_CM, 1, 80); check(t.worstCaseSeconds() < 26, "support rejects over-budget distance");
        check(t.notice().contains("rejected"), "budget warning");
        set(t, DRIVE_POWER, -1, 1); near(t.value(DRIVE_POWER), .45, "reject slower over-budget power");
        t.sequence();
        PollenShootTuning remembered = new PollenShootTuning(t.snapshot(), true);
        near(remembered.value(DISTANCE_CM), t.value(DISTANCE_CM), "support memory");
        check(remembered.profile() == mat, "remember calibration profile");
        select(t, CALIBRATION); t.lock(); tap(t, -1); check(t.profile() == mat, "lock calibration at START");
        PollenShootTuning shortDelay = support(); set(shortDelay, START_DELAY, -1, 40);
        near(shortDelay.value(START_DELAY), 4, "feeding needs four-second prespin window");
        set(shortDelay, DISTANCE_CM, 1, 80);
        remembered = new PollenShootTuning(shortDelay.snapshot(), true);
        near(remembered.value(START_DELAY), 15, "delay resets to support default");
        near(remembered.value(DISTANCE_CM), PollenFieldGeometry.supportDistanceCm(), "invalid memory resets to valid field preset");
        shortDelay.update(false, false, false, false, true);
        near(shortDelay.value(START_DELAY), 15, "triangle support wait");
        near(shortDelay.value(FEED_SECONDS), 7.5, "triangle support feed");
        near(shortDelay.value(DISTANCE_CM), PollenFieldGeometry.supportDistanceCm(), "triangle support field distance");
        check(new PollenShootTuning(null).value(FEED_SECONDS) == 10, "B defaults independent of support");

        // Square gives B a two-leg route without changing triangle's tested default.
        PollenShootTuning primary = new PollenShootTuning(null);
        primary.update(false, false, false, false, false, true); release(primary);
        near(primary.value(CLEARANCE_CM), 63.5, "field preset clears frame first");
        near(primary.value(DISTANCE_CM), PollenFieldGeometry.primaryDistanceCm(), "then travel to near parking slot");
        near(primary.value(DRIVE_POWER), .25, "primary field trial power");
        check(primary.worstCaseSeconds() < 26, "field preset fits full shot budget");
        s = primary.sequence(false); s.start(0); o = s.tick(0, 0, false);
        near(o.forward, 0, "clearance begins with left only"); near(o.strafe, -.25, "clearance left");
        zero(s.tick(primary.clearanceMove().seconds, 0, false));
        o = s.tick(primary.clearanceMove().seconds + .01, 0, false);
        near(o.forward, primary.forward(), "second leg after clearance"); near(o.strafe, primary.strafe(), "second leg angle");
        set(primary, DISTANCE_CM, -1, 100); near(primary.value(DISTANCE_CM), 0, "zero main leg");
        check(primary.clearanceMove().seconds > 0, "clearance remains independently enabled");
        s = primary.sequence(false); s.start(0); near(s.tick(0, 0, false).strafe, -.25, "zero main leg still performs selected clearance");
        primary.update(false, false, false, false, true); release(primary);
        near(primary.value(CLEARANCE_CM), 0, "triangle removes dogleg");
        near(primary.value(TRAVEL_ANGLE), 25.87, "triangle baseline angle");
        near(primary.driveSeconds(), 2.3627906976744186, "triangle original duration");
        select(primary, CALIBRATION); tap(primary, 1);
        remembered = new PollenShootTuning(primary.snapshot()); check(remembered.profile() == mat, "B profile memory");
        primary.lock(); primary.update(false, false, false, false, false, true);
        near(primary.value(CLEARANCE_CM), 0, "START locks field-preset button");
        System.out.println("PollenMatSupportTest: " + checks + " checks passed");
    }
}
