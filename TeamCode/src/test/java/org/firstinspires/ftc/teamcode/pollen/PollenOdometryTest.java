package org.firstinspires.ftc.teamcode.pollen;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.pollen.PollenShootTuning.Field.*;

/** Odometry autos, off the robot: the controller, the step order (shoot, wall square, zero,
 * legs), budgets, give-up and lost-pose handling, and full runs against a SIMULATED robot.
 * The simulation (speed, lag, strafe loss, yaw drift, wall) is a stand-in to exercise the logic;
 * its numbers are assumptions, not measurements of the real robot. */
public final class PollenOdometryTest {
    private static int checks;

    private static void check(boolean ok, String what) {
        checks++;
        if (!ok) throw new AssertionError(what);
    }

    /** Simple robot on a field with the end wall at x = 0 behind it. Inches, radians. */
    static final class SimRobot {
        double speedInPerSec = 140 / 2.54;   // per unit power, forward
        double strafeEfficiency = 0.8;      // mecanum strafes slower than it drives
        double turnRadPerSec = 3.0;         // per unit turn power
        double yawPerStrafe = 0.08;         // unwanted rotation while strafing, rad/s per unit
        double lagSeconds = 0.10;           // velocity follows command with this time constant
        double wallAlignRadPerSecPerPower = 1.0; // how fast pushing into the wall squares it
        boolean stuck;                      // nothing moves (e.g. blocked by another robot)
        double leftLimit = Double.POSITIVE_INFINITY; // field y it cannot pass (an obstacle on its left)
        double x, y, h, vf, vl, w;
        double ox, oy, oh;                  // where odometry was last zeroed

        void step(PollenSequence.Output out, double dt) {
            if (out.zeroOdometry) { ox = x; oy = y; oh = h; }
            double a = dt / (lagSeconds + dt);
            vf += a * (out.forward * speedInPerSec - vf);
            vl += a * (-out.strafe * speedInPerSec * strafeEfficiency - vl);
            w += a * (-out.turn * turnRadPerSec + yawPerStrafe * out.strafe - w);
            if (stuck) { vf = vl = w = 0; return; }
            double c = Math.cos(h), s = Math.sin(h);
            double nx = x + (c * vf - s * vl) * dt, ny = y + (s * vf + c * vl) * dt;
            if (nx < 0 && out.forward < 0) {
                // Pushed into the wall: it stops the robot and turns it flat against itself.
                double turnBy = Math.min(Math.abs(h), wallAlignRadPerSecPerPower * -out.forward * dt);
                h -= Math.signum(h) * turnBy;
                nx = 0;
                vf = 0;
                w = 0;
            }
            if (ny > leftLimit) { ny = leftLimit; vl = 0; }
            x = nx; y = ny; h += w * dt;
        }

        /** What the Pinpoint would report: pose relative to the last zero. */
        double[] odometry() {
            double dx = x - ox, dy = y - oy, c = Math.cos(-oh), s = Math.sin(-oh);
            return new double[] {c * dx - s * dy, s * dx + c * dy, PollenOdometryDrive.wrap(h - oh)};
        }
    }

    static final class Run {
        List<String> steps = new ArrayList<>();
        String result = "";
        String warning = "";
        double finishedAt = Double.NaN, feedAt = Double.NaN;
        List<String> trackPhase = new ArrayList<>();
        List<double[]> track = new ArrayList<>();
        SimRobot robot;
    }

    /** Ticks a sequence against the simulated robot every 20 ms until it finishes. */
    static Run simulate(PollenSequence seq, SimRobot robot) {
        Run run = new Run();
        run.robot = robot;
        double dt = 0.02;
        seq.start(0);
        for (double t = 0; t < 40; t += dt) {
            double[] p = robot.odometry();
            PollenSequence.Output out = seq.tick(t, p[0], p[1], p[2]);
            if (run.steps.isEmpty() && !out.phase.equals("START DELAY"))
                run.steps.add(out.phase.startsWith("SPINUP") ? "SHOOT" : out.phase); // first step
            if (out.phase.startsWith("NEXT: ")) run.steps.add(out.phase.substring(6));
            if (out.phase.equals("FEED") && Double.isNaN(run.feedAt)) run.feedAt = t;
            if (out.finished) { run.result = out.phase; run.finishedAt = t; break; }
            run.trackPhase.add(out.phase);
            run.track.add(new double[] {robot.x, robot.y});
            robot.step(out, dt);
        }
        run.warning = seq.warning();
        return run;
    }

    static SimRobot startCrooked(double degrees) {
        SimRobot r = new SimRobot();
        r.h = Math.toRadians(degrees);
        return r;
    }

    static void tap(PollenShootTuning t, boolean up, boolean down, boolean left, boolean right) {
        t.update(up, down, left, right, false);
        t.update(false, false, false, false, false);
    }

    static void setRow(PollenShootTuning t, PollenShootTuning.Field f, double value) {
        while (t.selected() != f) tap(t, false, true, false, false);
        for (int i = 0; i < 100 && t.value(f) != value; i++) tap(t, false, false, t.value(f) > value, t.value(f) < value);
        check(t.value(f) == value, "could not set " + f);
    }

    /** Distance from where the robot really is (wall frame) to where the last leg should end. */
    static double endError(Run run, double[] targetWallFrame) {
        return Math.hypot(run.robot.x - targetWallFrame[0], run.robot.y - targetWallFrame[1]);
    }

    /** For one leg (wall frame, from {@code from} to {@code to}): how far the robot strayed
     * sideways from the straight line, and how far it went past the end. Inches. */
    static double[] legDeviation(Run run, String leg, double[] from, double[] to) {
        double dx = to[0] - from[0], dy = to[1] - from[1], len = Math.hypot(dx, dy);
        double side = 0, past = 0;
        for (int i = 0; i < run.track.size(); i++) {
            String ph = run.trackPhase.get(i);
            if (!ph.equals(leg) && !ph.equals("NEXT: " + leg)) continue;
            double px = run.track.get(i)[0] - from[0], py = run.track.get(i)[1] - from[1];
            side = Math.max(side, Math.abs(px * dy - py * dx) / len);
            past = Math.max(past, (px * dx + py * dy) / len - len);
        }
        return new double[] {side, past};
    }

    static void speedRuns() {
        for (double power : new double[] {0.6, 0.8}) {
            for (double lag : new double[] {0.10, 0.20}) {
                PollenShootTuning c = new PollenShootTuning(null, true, true);
                setRow(c, DRIVE_POWER, power);
                SimRobot r = startCrooked(-2);
                r.lagSeconds = lag;
                Run run = simulate(c.sequence(), r);
                check(run.result.startsWith("DONE"), "C at " + power + " finishes: " + run.result + " " + run.warning);
                double[] l1 = legDeviation(run, "LEG 1 CLEAR FLOWER", new double[] {0, 0}, c.legEndInches(1));
                double[] l3 = legDeviation(run, "LEG 3 TO PARK", c.legEndInches(1), c.legEndInches(3));
                check(l1[0] < 1.0 && l3[0] < 1.0, "C at " + power + " stays within 1 in of the straight legs");
                check(endError(run, c.legEndInches(3)) < 0.75, "C at " + power + " ends on target");
                System.out.printf("  C at %.1f power, lag %.2f s: finish %.2f s (planned %.2f); leg 1 side %.2f past %.2f in;"
                                + " leg 3 side %.2f past %.2f in; end error %.2f in%n", power, lag, run.finishedAt,
                        c.worstCaseSeconds(), l1[0], l1[1], l3[0], l3[1], endError(run, c.legEndInches(3)));

                PollenShootTuning b = new PollenShootTuning(null, false, true);
                setRow(b, DRIVE_POWER, power);
                SimRobot rb = startCrooked(2);
                rb.lagSeconds = lag;
                Run runB = simulate(b.sequence(), rb);
                check(runB.result.startsWith("DONE") && endError(runB, b.legEndInches(3)) < 0.75,
                        "B field preset at " + power + " ends on target");
                double worstPast = 0;
                for (int leg = 1; leg <= 3; leg++) worstPast = Math.max(worstPast, legDeviation(runB, b.legName(leg),
                        leg == 1 ? new double[] {0, 0} : b.legEndInches(leg - 1), b.legEndInches(leg))[1]);
                System.out.printf("  B field preset at %.1f, lag %.2f s: finish %.2f s, worst overshoot %.2f in, end error %.2f in%n",
                        power, lag, runB.finishedAt, worstPast, endError(runB, b.legEndInches(3)));
            }
        }
    }

    public static void main(String[] args) {
        controller();
        stepOrderAndRows();
        shootFirstRuns();
        supportRun();
        fieldPresetRun();
        parkRun();
        failures();
        speedRuns();
        System.out.println("PollenOdometryTest: " + checks + " checks passed (simulated robot, not robot validation)");
    }

    static void controller() {
        double[] far = PollenOdometryDrive.command(48, 0, 0.4, 0, 0, 0);
        check(Math.abs(far[0] - 0.4) < 1e-9 && far[1] == 0 && far[2] == 0, "far ahead: full forward power");
        double[] left = PollenOdometryDrive.command(0, 48, 0.4, 0, 0, 0);
        check(Math.abs(left[1] + 0.4) < 1e-9, "target to the left: strafe is negative (strafe + is right)");
        // Robot turned 90 deg counterclockwise: a field-forward target is to its RIGHT.
        double[] turned = PollenOdometryDrive.command(48, 0, 0.4, 0, 0, Math.PI / 2);
        check(Math.abs(turned[0]) < 1e-9 && turned[1] > 0.39, "rotated robot strafes right toward field-forward target");
        check(turned[2] == PollenOdometryDrive.MAX_TURN, "rotated CCW: turns clockwise (capped)");
        double[] ccw = PollenOdometryDrive.command(48, 0, 0.4, 0, 0, Math.toRadians(5));
        check(ccw[2] > 0 && ccw[2] < 0.1, "5 deg CCW: small clockwise correction");
        double[] near = PollenOdometryDrive.command(1, 0, 0.4, 0, 0, 0);
        check(Math.abs(near[0] - 0.2) < 1e-9, "1 in out: slowed to 0.2");
        double[] nearer = PollenOdometryDrive.command(0.6, 0, 0.4, 0, 0, 0);
        check(Math.abs(nearer[0] - Math.max(PollenOdometryDrive.MIN_POWER, 0.2 * Math.sqrt(0.6))) < 1e-9,
                "just outside tolerance: never below the minimum power");
        double[] done = PollenOdometryDrive.command(0.3, 0.2, 0.4, 0, 0, 0.1);
        check(done[0] == 0 && done[1] == 0 && done[2] == 0, "inside tolerance: all zero");
        double[] end = PollenOdometryDrive.legEnd(0, 0, 2.54 * 10, 90);
        check(Math.abs(end[0]) < 1e-9 && Math.abs(end[1] - 10) < 1e-9, "leg at +90 deg ends to the left");
        check(PollenOdometryDrive.timeoutSeconds(100, .25) > PollenOdometryDrive.plannedSeconds(100, .25),
                "give-up time exceeds the plan");
    }

    static void stepOrderAndRows() {
        PollenShootTuning timed = new PollenShootTuning(null);
        check(!timed.odometry() && !timed.wallSquare() && timed.label(CALIBRATION).equals(CALIBRATION.label),
                "timed B unchanged: no odometry, original row label");
        check(simulate(timed.sequence(), new SimRobot()).steps.indexOf("ZERO ODOMETRY") < 0,
                "timed B has no zero step");

        PollenShootTuning b = new PollenShootTuning(null, false, true);
        check(b.legCm(1) > 0 && b.legCm(2) > 0 && Math.abs(b.legCm(3) - 217.47) < 0.05,
                "odometry B default is the full route to the planned park spot");
        check(b.wallSquare() && b.label(CALIBRATION).startsWith("Wall square"), "odometry B: wall square on by default");
        check(b.value(DRIVE_POWER) == PollenOdometryDrive.DEFAULT_LEG_POWER, "odometry B: default leg power");
        b.update(false, false, false, false, false); // Circle and Square are not inputs any more
        check(b.route() == PollenShootTuning.Route.FULL_PARK && b.route().label.startsWith("FULL PARK"),
                "B shows FULL PARK on the route line");
        setRow(b, FLYWHEEL, 1220);
        PollenShootTuning next = new PollenShootTuning(b.snapshot(), false, true);
        check(next.route() == PollenShootTuning.Route.FULL_PARK && next.value(FLYWHEEL) == 1220,
                "remembered settings keep the route and the edited launcher speed");
        check(new PollenShootTuning(null, false, true).route() == PollenShootTuning.Route.FULL_PARK
                && new PollenShootTuning(null, true, true).route() == PollenShootTuning.Route.SUPPORT_PARK,
                "default route lines: B full park, C support park");
        PollenShootTuning edited = new PollenShootTuning(null, false, true);
        setRow(edited, LEG2_CM, edited.value(LEG2_CM) + 5);
        check(edited.route() == PollenShootTuning.Route.EDITED, "editing a leg row shows EDITED LEGS");
        check(new PollenShootTuning(edited.snapshot(), false, true).route() == PollenShootTuning.Route.EDITED,
                "edited legs are remembered and still flagged");
        edited.update(false, false, false, false, true);
        check(edited.route() == PollenShootTuning.Route.FULL_PARK, "Triangle restores FULL PARK");
        check(edited.settingsLine().startsWith("route=FULL_PARK") && edited.settingsLine().contains(" LEG3_CM="),
                "log line lists the route and every row");
        check(b.value(DRIVE_POWER) == PollenOdometryDrive.DEFAULT_LEG_POWER, "odometry legs default to 0.6 power");
        setRow(b, DRIVE_POWER, 0.8);
        check(b.value(DRIVE_POWER) == 0.8, "odometry power row reaches 0.8");
        for (int i = 0; i < 3; i++) tap(b, false, false, false, true);
        check(b.value(DRIVE_POWER) == 0.8, "odometry power row stops at 0.8");
        setRow(b, DRIVE_POWER, 0.6);
        Run run = simulate(b.sequence(), new SimRobot());
        check(run.steps.toString().equals("[SQUARE TO WALL, ZERO ODOMETRY, SHOOT, LEG 1 OFF WALL, LEG 2 CLEAR HIVE, LEG 3 TO PARK]"),
                "B order: square, zero, shoot, legs; got " + run.steps);
        check(Math.abs(run.feedAt - b.feedStartSeconds()) < 0.1 && Math.abs(b.feedStartSeconds() - 3.45) < 1e-9,
                "B (no delay): squaring delays the feed by 0.45 s (+ step-change loops), to " + run.feedAt);
        setRow(b, CALIBRATION, 0);
        run = simulate(b.sequence(), new SimRobot());
        check(run.steps.toString().equals("[ZERO ODOMETRY, SHOOT, LEG 1 OFF WALL, LEG 2 CLEAR HIVE, LEG 3 TO PARK]"),
                "wall square off: zero only; got " + run.steps);
        check(simulate(timed.sequence(), new SimRobot()).steps.toString()
                        .equals("[SHOOT, LEG 1 OFF WALL, LEG 2 CLEAR HIVE, LEG 3 TO PARK, PARK / HOLD]"),
                "timed B default: shoot, the three timed legs, hold");
        Run full = simulate(new PollenShootTuning(null, false, true).sequence(), new SimRobot());
        check(full.steps.toString().equals("[SQUARE TO WALL, ZERO ODOMETRY, SHOOT, LEG 1 OFF WALL, LEG 2 CLEAR HIVE, LEG 3 TO PARK]"),
                "odometry B default order: " + full.steps);

        PollenShootTuning c = new PollenShootTuning(null, true, true);
        check(Math.abs(c.legCm(3) - 117.96) < 0.05 && Math.abs(c.legAngle(3) + 81.85) < 0.01,
                "odometry C leg 3 is the CAD plan (118.0 cm @ -81.85), not field-tuned 97.96");
        check(c.budgetOk(), "odometry C defaults fit the budget: " + c.worstCaseSeconds());
        PollenShootTuning remembered = new PollenShootTuning(c.snapshot(), true, true);
        check(remembered.legCm(3) == c.legCm(3) && remembered.value(START_DELAY) == PollenShootTuning.SUPPORT_START_DELAY,
                "odometry C snapshot restores legs and resets delay");
    }

    static void shootFirstRuns() {
        PollenShootTuning b = new PollenShootTuning(null, false, true);
        double[] target = b.legEndInches(3);
        Run squared = simulate(b.sequence(), startCrooked(3));
        check(squared.result.startsWith("DONE"), "B squared finishes: " + squared.result);
        check(squared.finishedAt < PollenShootTuning.DEADLINE, "B finishes before the deadline");
        check(endError(squared, target) < 0.75, "B squared ends within 0.75 in of target: " + endError(squared, target));
        check(Math.abs(Math.toDegrees(squared.robot.h)) < 1, "B squared ends square to the wall");

        // The comparison uses the long field-preset route, where a crooked start matters most.
        PollenShootTuning field = new PollenShootTuning(null, false, true);
        double[] fieldTarget = field.legEndInches(3);
        Run fieldSquared = simulate(field.sequence(), startCrooked(3));
        setRow(field, CALIBRATION, 0);
        Run crooked = simulate(field.sequence(), startCrooked(3));
        check(endError(fieldSquared, fieldTarget) < 0.75, "field preset squared ends on target");
        check(endError(crooked, fieldTarget) > 4.0, "without squaring a 3 deg start misses the 98 in route by > 4 in: "
                + endError(crooked, fieldTarget));
        System.out.printf("  B field preset, start 3 deg crooked: end error %.2f in with wall square, %.2f in without%n",
                endError(fieldSquared, fieldTarget), endError(crooked, fieldTarget));
    }

    static void supportRun() {
        PollenShootTuning c = new PollenShootTuning(null, true, true);
        Run run = simulate(c.sequence(), startCrooked(-2));
        check(run.result.startsWith("DONE"), "C finishes: " + run.result + " " + run.warning);
        check(Math.abs(run.feedAt - 15) < 0.1 && c.feedStartSeconds() == 15,
                "C squares inside its delay and still feeds at 15 s: " + run.feedAt);
        check(run.steps.toString().equals("[SQUARE TO WALL, ZERO ODOMETRY, SHOOT, LEG 1 CLEAR FLOWER, LEG 3 TO PARK]"),
                "C order: " + run.steps);
        check(endError(run, c.legEndInches(3)) < 0.75, "C ends on target: " + endError(run, c.legEndInches(3)));
        System.out.printf("  C: planned %.2f s, simulated finish %.2f s (deadline %.0f s)%n",
                c.worstCaseSeconds(), run.finishedAt, PollenShootTuning.DEADLINE);
        // A slower robot than the estimate: the deadline still stops everything.
        SimRobot slow = startCrooked(0);
        slow.speedInPerSec = 60 / 2.54;
        Run late = simulate(c.sequence(), slow);
        check(late.finishedAt <= PollenShootTuning.DEADLINE + 0.05, "slow robot still stops by the deadline");
    }

    static void fieldPresetRun() {
        PollenShootTuning b = new PollenShootTuning(null, false, true);
        // Default: the full 3-leg route (formerly the Square preset).
        check(b.legCm(1) > 0 && b.legCm(2) > 0 && b.wallSquare(), "B field preset: three legs, squaring on");
        check(b.budgetOk(), "B field preset fits the budget: " + b.worstCaseSeconds());
        Run run = simulate(b.sequence(), startCrooked(2));
        check(run.result.startsWith("DONE"), "B field preset finishes: " + run.result + " " + run.warning);
        check(endError(run, b.legEndInches(3)) < 0.75, "B field preset ends on the planned spot");
        System.out.printf("  B field preset: ends %.1f in fwd, %.1f in left of the squared start; finish %.2f s%n",
                run.robot.x, run.robot.y, run.finishedAt);
    }

    static void parkRun() {
        PollenOdoParkTuning a = new PollenOdoParkTuning(.20, 0, 27, .5, null);
        check(a.cm() == PollenOdoParkTuning.DEFAULT_CM && a.power() == .20, "A defaults: 55 cm at the given power");
        a.update(false, false, false, true, false, false, false);
        a.update(false, false, false, false, false, false, false);
        check(a.cm() == 60, "A RIGHT adds 5 cm");
        a.update(false, false, false, false, false, false, true);
        check(a.cm() == 55, "A Triangle resets");
        Run run = simulate(a.sequence(), startCrooked(0));
        check(run.steps.toString().equals("[ZERO ODOMETRY, FORWARD TO PARK, PARK / HOLD]"), "A order: " + run.steps);
        check(Math.abs(run.robot.x - 55 / 2.54) < 0.6 && Math.abs(run.robot.y) < 0.6, "A ends 55 cm ahead: x " + run.robot.x + " y " + run.robot.y + " want " + 55 / 2.54);
        PollenOdoParkTuning kept = new PollenOdoParkTuning(.20, 0, 27, .5, a.snapshot());
        check(kept.cm() == a.cm() && kept.power() == a.power(), "A snapshot keeps distance and power");
        check(a.maximumDelaySeconds() >= 20, "A default leaves the full 20 s delay range");
    }

    static void failures() {
        PollenShootTuning b = new PollenShootTuning(null, false, true);
        SimRobot stuck = new SimRobot();
        stuck.stuck = true;
        Run run = simulate(b.sequence(), stuck);
        check(run.result.equals("LEG 1 OFF WALL NOT MOVING - STOPPED"), "robot that cannot move stops: " + run.result);
        check(!run.steps.contains("LEG 2 CLEAR HIVE"), "and never starts the next leg: " + run.steps);
        check(run.finishedAt - run.feedAt < 13 + 1.5, "stops within about a second of starting to drive: " + run.finishedAt);

        // The review's case: the clearance leg is blocked part way (5 in into the 25 in side move).
        SimRobot blocked = new SimRobot();
        blocked.leftLimit = 5;
        run = simulate(new PollenShootTuning(null, false, true).sequence(), blocked);
        check(run.result.equals("LEG 2 CLEAR HIVE NOT MOVING - STOPPED"), "blocked clearance leg stops the route: " + run.result);
        check(!run.steps.contains("LEG 3 TO PARK"), "leg 3 never starts from the blocked spot: " + run.steps);
        check(run.warning.contains("short"), "warning says how far short: " + run.warning);

        PollenSequence seq = new PollenShootTuning(null, false, true).sequence();
        seq.start(0);
        String result = "";
        for (double t = 0; t < 30; t += 0.02) {
            PollenSequence.Output out = seq.tick(t, Double.NaN, Double.NaN, Double.NaN);
            if (out.finished) { result = out.phase; break; }
        }
        check(result.equals("NO ODOMETRY POSE - STOPPED"), "lost pose during a leg stops everything: " + result);
    }
}
