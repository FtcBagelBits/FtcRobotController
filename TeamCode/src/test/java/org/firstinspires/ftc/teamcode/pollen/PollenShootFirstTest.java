package org.firstinspires.ftc.teamcode.pollen;

import static org.firstinspires.ftc.teamcode.pollen.PollenShootTuning.Field.*;

/** Behavioral checks for shoot-first B's shot, INIT menu and budget, using a one-leg route made
 * with the D-pad (legs 1 and 2 at 0 cm, baseline calibration, power 0.20); no FTC/Android
 * dependencies. B's default full route is covered by the other tests. */
public final class PollenShootFirstTest {
    private static int checks;
    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static void near(double got, double expected, String message) {
        check(Math.abs(got - expected) < .000001, message + ": " + got + " != " + expected);
    }
    private static void zero(PollenSequence.Output out) {
        check(out.forward == 0 && out.strafe == 0 && out.turn == 0
                && out.launcherVelocity == 0 && out.feederPower == 0, "zero outputs");
    }
    private static void release(PollenShootTuning t) { t.update(false, false, false, false, false); }
    private static void triangle(PollenShootTuning t) { t.update(false, false, false, false, true); release(t); }
    /** B's default route edited down to leg 3 only, baseline calibration, power 0.20. */
    private static PollenShootTuning leg3Only() {
        PollenShootTuning t = new PollenShootTuning(null);
        select(t, LEG1_CM); for (int i = 0; i < 100; i++) change(t, -1);
        select(t, LEG2_CM); for (int i = 0; i < 100; i++) change(t, -1);
        select(t, CALIBRATION); change(t, -1);
        select(t, DRIVE_POWER); change(t, -1);
        return t;
    }
    private static void select(PollenShootTuning t, PollenShootTuning.Field field) {
        while (t.selected() != field) {
            t.update(false, true, false, false, false);
            release(t);
        }
    }
    private static void change(PollenShootTuning t, int direction) {
        t.update(false, false, direction < 0, direction > 0, false);
        release(t);
    }
    /** Tick every 20 ms from {@code from} up to (not including) {@code to}; returns the last output. */
    private static PollenSequence.Output run(PollenSequence s, double from, double to) {
        PollenSequence.Output out = null;
        for (double now = from; now < to - 1e-9; now += .02) out = s.tick(now);
        return out;
    }
    public static void main(String[] args) {
        PollenShootTuning full = new PollenShootTuning(null);
        check(full.legCm(1) > 0 && full.legCm(2) > 0 && full.value(LEG3_CM) == PollenShootTuning.B_FIELD_LEG3_CM,
                "B default is now the full 3-leg route to the parking slot");
        PollenShootTuning t = leg3Only();
        check(t.legCm(1) == 0 && t.legCm(2) == 0 && t.route() == PollenShootTuning.Route.EDITED, "one-leg route, flagged EDITED");
        near(t.value(LEG3_CM), PollenShootTuning.B_FIELD_LEG3_CM, "leg 3 keeps the field-tuned distance");
        near(t.value(DRIVE_POWER), .20, "power 0.20");
        check(t.driveSeconds() > 0, "leg 3 has a travel time");
        near(t.value(FEED_SECONDS), 10, "ten seconds excludes spinup");
        near(t.value(FEED_POWER), .5, "intake matches the neutral-trigger TeleOp feed boost");
        near(t.value(AGITATOR_POWER), 1, "agitator runs at full power while feeding");
        near(t.forward() * t.forward() + t.strafe() * t.strafe(), .04, "vector magnitude");
        check(t.forward() > 0 && t.strafe() < 0, "forward-left departure");
        near(t.forward(), .20 * Math.cos(Math.toRadians(17)), "fixed forward component");
        near(t.strafe(), -.20 * Math.sin(Math.toRadians(17)), "fixed left component");

        PollenSequence s = t.sequence();
        zero(s.tick(0));
        s.start(0);
        PollenSequence.Output spin = s.tick(0);
        near(spin.launcherVelocity, 1180, "flywheel starts alone");
        near(spin.feederPower, 0, "no premature feed");
        near(spin.forward, 0, "no approach drive");
        near(t.value(SPINUP), 3, "three-second spin-up default");
        near(s.tick(1).feederPower, 0, "launcher alone during spin-up, even at full speed");
        near(s.tick(2.99).feederPower, 0, "still spinning up just before 3 s");
        near(s.tick(3).feederPower, 1, "feed at 3 s even with zero measured speed");
        PollenSequence.Output dip = s.tick(4);
        near(dip.feederPower, 1, "ordinary speed dip does not pause continuous feed");
        near(dip.launcherVelocity, 1180, "flywheel target maintained during feed");
        near(dip.forward, 0, "stationary shooting");
        near(s.tick(12.99).feederPower, 1, "full ten seconds after spin-up");
        zero(s.tick(13));
        PollenSequence.Output move = s.tick(13.01);
        near(move.forward, t.forward(), "depart after feeding");
        near(move.strafe, t.strafe(), "diagonal departure");
        near(move.turn, 0, "does not rotate");
        near(move.launcherVelocity, 0, "flywheel off during travel");
        near(move.feederPower, 0, "both feeds off during travel");
        for (double p : PollenSequence.wheelPowers(move)) check(Math.abs(p) <= 1, "bounded wheel power");
        // The leg's clock started at 13.01, the first loop that commanded it.
        double travelEnds = 13.01 + t.driveSeconds();
        PollenSequence.Output last = run(s, 13.03, travelEnds - .005);
        check(last.phase.equals("LEG 3 TO PARK") && last.forward == t.forward(), "still driving at loop rate");
        PollenSequence.Output end = s.tick(travelEnds + .001);
        zero(end); check(end.phase.startsWith("NEXT"), "leg ends on its own time");
        check(!run(s, travelEnds + .021, travelEnds + .5).finished, "hold lasts half a second");
        PollenSequence.Output done = s.tick(travelEnds + .522);
        check(done.finished && done.phase.startsWith("DONE"), "hold then finish, not a stall stop"); zero(done);

        s = t.sequence(); s.start(0);
        s.tick(0); near(s.tick(3).feederPower, 1, "no speed reading: still feeds");
        zero(s.tick(13));
        check(s.warning().isEmpty(), "no speed warning: speed is not checked");
        near(s.tick(13.01).strafe, t.strafe(), "then the selected diagonal, not Park A");
        s = t.sequence(false); s.start(0);
        near(s.tick(0).strafe, t.strafe(), "INIT fallback uses same departure");
        for (double stopAt : new double[]{0, .5, 3, 11.5, 12}) {
            s = t.sequence(); s.start(0);
            for (double now = 0; now < stopAt; now += .02) s.tick(now);
            zero(s.stop("STOPPED"));
            PollenSequence.Output stopped = s.tick(stopAt + 1);
            zero(stopped); check(stopped.finished, "STOP latches in every phase");
        }
        s = t.sequence(); s.start(0); s.tick(1);
        check(s.tick(.5).finished, "backward clock stops");
        s = t.sequence(); s.start(0);
        check(s.tick(27).finished, "independent deadline");

        // UI edges, limits, budget rejection, session memory and locking.
        select(t, FLYWHEEL);
        change(t, 1); near(t.value(FLYWHEEL), 1200, "speed adjustment: one 20 tick/s step");
        t.update(false, false, false, true, false);
        t.update(false, false, false, true, false);
        near(t.value(FLYWHEEL), 1220, "held key does not repeat"); release(t);
        t.update(false, false, true, true, false);
        near(t.value(FLYWHEEL), 1220, "opposite arrows cancel"); release(t);
        select(t, START_DELAY); change(t, 1);
        PollenShootTuning remembered = new PollenShootTuning(t.snapshot());
        near(remembered.value(FLYWHEEL), 1220, "remember speed");
        near(remembered.value(START_DELAY), 0, "reset delay on INIT");
        near(t.value(START_DELAY), .5, "snapshots do not alias");
        for (int i = 0; i < 60; i++) change(t, 1);
        check(t.worstCaseSeconds() < 26, "budget bounded");
        check(t.notice().contains("rejected"), "budget rejection visible");
        s = t.sequence(); s.start(0); zero(s.tick(t.value(START_DELAY) - .01));
        near(s.tick(t.value(START_DELAY)).launcherVelocity, 1220, "delay precedes spinup");
        t.lock(); double before = t.value(START_DELAY);
        change(t, -1); t.update(false, false, false, false, true);
        near(t.value(START_DELAY), before, "START locks edits and reset");

        t = leg3Only();
        select(t, FEED_SECONDS); for (int i = 0; i < 25; i++) change(t, -1);
        near(t.value(FEED_SECONDS), 0, "skip shot for motion calibration");
        s = t.sequence(); s.start(0); near(s.tick(0).forward, t.forward(), "skip-shot route");
        select(t, LEG3_CM); for (int i = 0; i < 100; i++) change(t, 1);
        near(t.value(LEG3_CM), 350, "distance upper bound");
        for (int i = 0; i < 100; i++) change(t, -1);
        near(t.value(LEG3_CM), 0, "zero distance for stationary test");
        s = t.sequence(); s.start(0); zero(s.tick(0));
        check(s.tick(.5).finished, "both zero gives hold only");
        triangle(t);
        near(t.value(LEG3_CM), PollenShootTuning.B_FIELD_LEG3_CM, "triangle restores leg 3");
        near(t.value(FEED_SECONDS), 10, "triangle restores feed duration");
        check(t.legCm(1) > 0 && t.legCm(2) > 0, "triangle restores the full route");
        t = leg3Only();
        select(t, LEG3_CM); for (int i = 0; i < 44; i++) change(t, 1);
        check(t.worstCaseSeconds() < 26, "longer selected distance remains within time budget");
        select(t, DRIVE_POWER); for (int i = 0; i < 10; i++) change(t, -1);
        check(t.worstCaseSeconds() < 26, "slowing drive cannot silently exhaust budget");
        t.sequence(); // Still a valid sequence after refused changes.
        // Angle editing changes translation only; retain the calibrated default.
        t = leg3Only();
        near(t.value(LEG3_ANGLE), 17, "field-tuned default angle preserved");
        double duration = t.driveSeconds(), budget = t.worstCaseSeconds();
        select(t, LEG3_ANGLE); change(t, 1);
        near(t.value(LEG3_ANGLE), 18, "angle steps by one degree");
        t.update(false, false, false, true, false);
        t.update(false, false, false, true, false);
        near(t.value(LEG3_ANGLE), 19, "held angle button does not repeat"); release(t);
        t.update(false, false, true, true, false);
        near(t.value(LEG3_ANGLE), 19, "opposite angle edits cancel"); release(t);
        for (int i = 0; i < 19; i++) change(t, -1);
        near(t.value(LEG3_ANGLE), 0, "can select exactly straight forward");
        near(t.forward(), .2, "zero angle forward output");
        near(t.strafe(), 0, "zero angle no strafe");
        change(t, -1);
        check(t.strafe() > 0, "negative angle moves right");
        near(t.driveSeconds(), duration, "angle does not change estimated travel duration");
        near(t.worstCaseSeconds(), budget, "angle does not change budget");
        remembered = new PollenShootTuning(t.snapshot());
        near(remembered.value(LEG3_ANGLE), -1, "angle survives next INIT in same app");
        for (int i = 0; i < 200; i++) change(t, -1);
        near(t.value(LEG3_ANGLE), -90, "right angle limit");
        near(t.forward(), 0, "right strafe no forward");
        near(t.strafe(), .2, "right strafe output");
        // Shooter fallback must use the selected angle, not the default angle.
        s = t.sequence(false); s.start(0);
        move = s.tick(0);
        near(move.strafe, .2, "fallback uses selected right strafe");
        near(move.turn, 0, "angle never turns chassis");
        zero(s.stop("STOPPED")); zero(s.tick(.2));
        for (int i = 0; i < 200; i++) change(t, 1);
        near(t.value(LEG3_ANGLE), 90, "left angle limit");
        near(t.forward(), 0, "left strafe no forward");
        near(t.strafe(), -.2, "left strafe output");
        s = t.sequence(); s.start(0);
        s.tick(0); s.tick(3);
        zero(s.tick(13));
        move = s.tick(13.01);
        near(move.strafe, -.2, "successful shot departs at selected angle");
        near(move.turn, 0, "successful shot departure does not turn");
        triangle(t);
        near(t.value(LEG3_ANGLE), 17, "triangle restores the field-tuned angle");
        // Fractional defaults (C's -81.85): the first edit reaches the next whole degree.
        PollenShootTuning c = new PollenShootTuning(null, true);
        select(c, LEG3_ANGLE); change(c, -1);
        near(c.value(LEG3_ANGLE), -82, "first left from a fractional angle reaches the next whole degree");
        c = new PollenShootTuning(null, true);
        select(c, LEG3_ANGLE); change(c, 1);
        near(c.value(LEG3_ANGLE), -81, "first right from a fractional angle reaches the next whole degree");
        select(t, LEG3_ANGLE); change(t, -1);
        near(t.value(LEG3_ANGLE), 16, "edit after Triangle");
        t.lock(); change(t, 1);
        t.update(false, false, false, false, true);
        near(t.value(LEG3_ANGLE), 16, "START locks angle edits and reset");
        System.out.println("PollenShootFirstTest: " + checks + " checks passed");
    }
}
