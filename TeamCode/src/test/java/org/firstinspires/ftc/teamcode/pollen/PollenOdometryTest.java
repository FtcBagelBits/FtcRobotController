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
        boolean cannotTurnInPlace;          // drives normally, but a turn in place does nothing (a corner against the wall)
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
            if (cannotTurnInPlace && out.turn != 0 && out.forward == 0 && out.strafe == 0) { vf = vl = w = 0; return; }
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
        /** Where the robot really was (wall frame: x, y, heading) on the first FEED loop. */
        double[] feedPose;
        List<String> trackPhase = new ArrayList<>();
        List<double[]> track = new ArrayList<>();
        SimRobot robot;
    }

    /** Ticks a sequence against the simulated robot every 20 ms until it finishes. The launcher
     * gives no speed reading, so shots feed when their spin-up time is over. */
    static Run simulate(PollenSequence seq, SimRobot robot) {
        return simulate(seq, robot, Double.NaN);
    }

    /** Same, with a launcher that reads up to speed once it has been on for
     * {@code launcherReadySeconds} (NaN = no speed reading at all). */
    static Run simulate(PollenSequence seq, SimRobot robot, double launcherReadySeconds) {
        Run run = new Run();
        run.robot = robot;
        double dt = 0.02;
        double launcherOnAt = Double.NaN, launcherTarget = 0;
        seq.start(0);
        for (double t = 0; t < 40; t += dt) {
            double[] p = robot.odometry();
            double launcherSpeed = Double.isNaN(launcherReadySeconds) ? Double.NaN
                    : !Double.isNaN(launcherOnAt) && t - launcherOnAt >= launcherReadySeconds ? launcherTarget : 0;
            PollenSequence.Output out = seq.tick(t, p[0], p[1], p[2], launcherSpeed);
            if (out.launcherVelocity == 0) launcherOnAt = Double.NaN;
            else if (Double.isNaN(launcherOnAt)) { launcherOnAt = t; launcherTarget = out.launcherVelocity; }
            if (run.steps.isEmpty() && !out.phase.equals("START DELAY"))
                run.steps.add(out.phase.startsWith("SPINUP") ? "SHOOT" : out.phase); // first step
            if (out.phase.startsWith("NEXT: ")) run.steps.add(out.phase.substring(6));
            if (out.phase.equals("FEED") && Double.isNaN(run.feedAt)) {
                run.feedAt = t;
                run.feedPose = new double[] {robot.x, robot.y, robot.h};
            }
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

    /** The feed rule in whole runs: the primary shooter feeds as soon as its launcher is up to
     * speed; the Supporting Shooter still feeds at the time agreed with the partner. */
    static void feedWhenUpToSpeed() {
        PollenShootTuning b = new PollenShootTuning(null, false, true);
        Run timer = simulate(b.sequence(), new SimRobot());
        Run ready = simulate(b.sequence(), new SimRobot(), 1.2);
        check(Math.abs(timer.feedAt - b.feedStartSeconds()) < 0.1, "B, no speed reading: feeds when the spin-up time is over, " + timer.feedAt);
        check(Math.abs((timer.feedAt - ready.feedAt) - (b.value(SPINUP) - 1.2)) < 0.05,
                "B, launcher ready after 1.2 s: feeds 1.8 s sooner, at " + ready.feedAt);
        check(Math.abs((timer.finishedAt - ready.finishedAt) - 1.8) < 0.1 && ready.result.startsWith("DONE"),
                "and the whole route finishes that much sooner: " + ready.finishedAt + " vs " + timer.finishedAt);
        check(b.feedsWhenUpToSpeed() && b.worstCaseSeconds() > timer.finishedAt - 2,
                "the time budget still plans for the whole spin-up time");
        Run slow = simulate(b.sequence(), new SimRobot(), 99);
        check(Math.abs(slow.feedAt - timer.feedAt) < 1e-9, "B, launcher never reads up to speed: the timer still feeds on time");

        PollenShootTuning c = new PollenShootTuning(null, true, true);
        Run agreed = simulate(c.sequence(), new SimRobot(), 0.5);
        check(!c.feedsWhenUpToSpeed() && Math.abs(c.feedStartSeconds() - 15) < 1e-9, "C plans its feed for 15 s");
        check(agreed.feedAt >= 15 - 1e-9 && agreed.feedAt < 15.1,
                "C, launcher ready after 0.5 s: still feeds at 15 s, never sooner; fed at " + agreed.feedAt);
        System.out.printf(java.util.Locale.US, "  Feed rule: B feeds at %.2f s with a launcher ready in 1.2 s (%.2f s on the timer); C at %.2f s%n",
                ready.feedAt, timer.feedAt, agreed.feedAt);
    }

    /** Runs a route of turns only (after a zero) and returns the run. */
    static Run simulateTurns(SimRobot robot, double... headingsDegrees) {
        List<PollenSequence.Step> steps = new ArrayList<>();
        steps.add(PollenSequence.Step.zeroOdometry(PollenOdometryDrive.ZERO_SETTLE_SECONDS));
        double from = 0;
        for (double h : headingsDegrees) {
            double power = PollenOdometryDrive.DEFAULT_TURN_POWER;
            steps.add(PollenSequence.Step.turnTo("TURN TO " + (int) h, h, power,
                    PollenOdometryDrive.turnPlannedSeconds(h - from, power),
                    PollenOdometryDrive.turnTimeoutSeconds(h - from, power)));
            from = h;
        }
        return simulate(PollenSequence.park(steps.toArray(new PollenSequence.Step[0]), 27, 0), robot);
    }

    /** Turning in place (Kind.TURN). */
    static void turns() {
        double max = PollenOdometryDrive.DEFAULT_TURN_POWER, min = PollenOdometryDrive.MIN_TURN_POWER;
        // The controller. Headings are counterclockwise-positive; turn power is clockwise-positive.
        check(PollenOdometryDrive.turnPower(0, max, Math.toRadians(30), 0) == max,
                "30 deg counterclockwise of the target: full power clockwise (+)");
        check(PollenOdometryDrive.turnPower(Math.toRadians(-36), max, 0, 0) == max,
                "target 36 deg to the right: turn clockwise (+)");
        check(PollenOdometryDrive.turnPower(Math.toRadians(36), max, 0, 0) == -max,
                "target 36 deg to the left: turn counterclockwise (-)");
        check(PollenOdometryDrive.turnPower(0, max, Math.toRadians(1.5), 0) == 0, "inside the tolerance: no power");
        check(PollenOdometryDrive.turnPower(0, max, Math.toRadians(2.1), 0) == min, "just outside the tolerance: the minimum power, not less");
        check(Math.abs(PollenOdometryDrive.turnPower(0, max, Math.toRadians(9), 0) - 0.30) < 1e-9, "9 deg out: slowed to 0.30");
        check(PollenOdometryDrive.turnPower(0, max, Math.toRadians(5), Math.toRadians(-60)) == 0,
                "5 deg out and closing at 60 deg/s: it will coast there, so no power");
        check(PollenOdometryDrive.turnPower(0, max, Math.toRadians(170), 0) == max
                        && PollenOdometryDrive.turnPower(Math.toRadians(170), max, Math.toRadians(-170), 0) == max,
                "takes the short way round through 180");
        check(PollenOdometryDrive.rampedTurnPower(0, max) == min && PollenOdometryDrive.rampedTurnPower(5, max) == max,
                "turn power ramps up from the minimum");

        // On the simulated robot: to the default aim angle and back, with different robots.
        double aim = PollenFieldGeometry.angledAimDegrees();
        double[][] robots = {{3.0, 0.10}, {3.0, 0.20}, {1.8, 0.10}, {4.5, 0.10}, {4.5, 0.25}}; // turn rad/s per power, lag s
        for (double[] kind : robots) {
            SimRobot robot = new SimRobot();
            robot.turnRadPerSec = kind[0];
            robot.lagSeconds = kind[1];
            Run run = simulateTurns(robot, aim, 0);
            String label = String.format(java.util.Locale.US, "robot turning %.1f rad/s per power, lag %.2f s", kind[0], kind[1]);
            check(run.result.startsWith("DONE") && run.warning.isEmpty(), label + ": both turns finish cleanly: " + run.result + " " + run.warning);
            check(Math.abs(Math.toDegrees(robot.h)) <= PollenOdometryDrive.HEADING_TOLERANCE_DEG + 0.5,
                    label + ": ends back at heading 0, off by " + Math.toDegrees(robot.h));
            double planned = 2 * PollenOdometryDrive.turnPlannedSeconds(aim, max);
            System.out.printf(java.util.Locale.US, "  Turn to %.1f deg and back, %s: %.2f s (planned %.2f), ends %.2f deg off%n",
                    aim, label, run.finishedAt - PollenOdometryDrive.ZERO_SETTLE_SECONDS, planned, Math.toDegrees(robot.h));
        }
        SimRobot one = new SimRobot();
        Run toAim = simulateTurns(one, aim);
        check(Math.abs(Math.toDegrees(one.h) - aim) <= PollenOdometryDrive.HEADING_TOLERANCE_DEG + 0.5,
                "one turn ends at the aim angle, off by " + (Math.toDegrees(one.h) - aim));
        check(Math.hypot(one.x, one.y) < 0.5, "turning in place does not move the robot: " + Math.hypot(one.x, one.y));
        check(toAim.finishedAt < PollenOdometryDrive.turnTimeoutSeconds(aim, max), "well inside its give-up time: " + toAim.finishedAt);

        // Failures. A robot that cannot turn (a corner against the wall): stop, do not shoot.
        SimRobot stuck = new SimRobot();
        stuck.stuck = true;
        Run run = simulateTurns(stuck, aim, 0);
        check(run.result.equals("TURN TO -36 NOT TURNING - STOPPED"), "robot that cannot turn stops the route: " + run.result);
        check(run.finishedAt < 1.2 && run.warning.contains("short"), "within about a second, and says how far short: " + run.finishedAt + " " + run.warning);

        // Scripted headings, for the two ways a turn runs out of time.
        PollenSequence.Step[] route = {PollenSequence.Step.turnTo("TURN", -36, max, 1, 2),
                PollenSequence.Step.drive("AFTER", 1, 0, 0, 0)};
        // Creeping, so not stuck, and still 10 deg short at the give-up time: STOP.
        PollenSequence far = PollenSequence.park(route, 27, 0);
        far.start(0);
        PollenSequence.Output out = null;
        for (double t = 0; t < 3 && (out == null || !out.finished); t += 0.02)
            out = far.tick(t, 0, 0, Math.toRadians(-13 * t));
        check(out.finished && out.phase.equals("TURN TIMED OUT - STOPPED") && far.warning().contains("ran out of time"),
                "a turn far short at its give-up time stops the route: " + out.phase);
        // 4 deg short and not moving: close enough, the next step runs, with a warning.
        PollenSequence close = PollenSequence.park(route, 27, 0);
        close.start(0);
        String last = "";
        for (double t = 0; t < 2.1; t += 0.02) last = close.tick(t, 0, 0, Math.toRadians(-32)).phase;
        check((last.equals("AFTER") || last.equals("NEXT: AFTER")) && close.warning().contains("close enough"),
                "a turn 4 deg short at its give-up time hands over with a warning: " + last + " / " + close.warning());
        // No heading at all: stop.
        PollenSequence blind = PollenSequence.park(route, 27, 0);
        blind.start(0);
        check(blind.tick(0, 0, 0, Double.NaN).phase.equals("NO ODOMETRY POSE - STOPPED"), "a turn with no heading stops everything");
        try {
            PollenSequence.Step.turnTo("BAD", Double.NaN, max, 1, 2);
            check(false, "a turn to no heading must be rejected");
        } catch (IllegalArgumentException expected) { checks++; }
    }

    /** The experimental Angled Primary Shooter. */
    static void angledShot() {
        PollenShootTuning angled = new PollenShootTuning(null, false, true, true);
        PollenShootTuning b = new PollenShootTuning(null, false, true);
        check(angled.angled() && angled.route() == PollenShootTuning.Route.ANGLED_PARK && angled.route().label.startsWith("ANGLED SHOT"),
                "angled mode shows ANGLED SHOT on the route line");
        check(angled.rows().length == b.rows().length + 1 && angled.rows()[angled.rows().length - 1] == AIM_ANGLE,
                "the aim angle row exists only in angled mode");
        for (PollenShootTuning.Field f : b.rows()) check(f != AIM_ANGLE, "no aim angle row in Odo Shoot + Park");
        check(!b.settingsLine().contains("AIM_ANGLE") && angled.settingsLine().contains(" AIM_ANGLE=-36"),
                "the log line has the aim angle only in angled mode: " + angled.settingsLine());

        // The geometry. The shooting spot is 5 in off the wall and 30 in to the left; from there
        // the hive opening (45.9 in straight ahead of the start) is about 36 deg to the right.
        double[] spot = angled.legEndInches(2);
        check(Math.abs(spot[0] - 5) < 0.01 && Math.abs(spot[1] - 30) < 0.01, "legs 1 and 2 end at the shooting spot: " + spot[0] + ", " + spot[1]);
        double aim = Math.toRadians(angled.aimDegrees()), range = PollenFieldGeometry.angledShotDistanceInches();
        check(Math.abs(angled.aimDegrees() + 36.26) < 0.1 && Math.abs(range - 50.7) < 0.1,
                "aim about -36 deg, about 51 in from the opening: " + angled.aimDegrees() + ", " + range);
        check(Math.abs(spot[0] + range * Math.cos(aim) - 45.9) < 0.05 && Math.abs(spot[1] + range * Math.sin(aim)) < 0.05,
                "the aim line from the spot ends at the hive opening, 45.9 in straight ahead of the start");
        double[] park = angled.legEndInches(3), parkB = b.legEndInches(3);
        check(Math.abs(park[0] - parkB[0]) < 0.05 && Math.abs(park[1] - parkB[1]) < 0.05,
                "leg 3 ends at the same parking spot as Odo Shoot + Park: " + park[0] + ", " + park[1]);
        check(angled.budgetOk() && angled.worstCaseSeconds() < 26, "defaults fit the time budget: " + angled.worstCaseSeconds());
        check(angled.roomToTurn() && angled.turnWallGapInches() > 1, "the corner clears the wall while turning: " + angled.turnWallGapInches());

        // A whole run on the simulated robot, started 2 deg crooked.
        Run run = simulate(angled.sequence(), startCrooked(2), 1.2);
        check(run.steps.toString().equals("[SQUARE TO WALL, ZERO ODOMETRY, LEG 1 OFF WALL, LEG 2 TO SHOOTING SPOT, "
                        + "TURN TO AIM, SHOOT, TURN BACK, LEG 3 TO PARK]"),
                "angled order: square, zero, 2 legs, turn, shoot, turn back, park; got " + run.steps);
        check(run.result.startsWith("DONE") && run.warning.isEmpty(), "angled run finishes cleanly: " + run.result + " " + run.warning);
        double spotError = Math.hypot(run.feedPose[0] - spot[0], run.feedPose[1] - spot[1]);
        double aimError = Math.toDegrees(run.feedPose[2]) - angled.aimDegrees();
        check(spotError < 1.5, "it shoots from the shooting spot, off by " + spotError + " in");
        check(Math.abs(aimError) <= PollenOdometryDrive.HEADING_TOLERANCE_DEG + 0.5, "aimed at the hive when feeding starts, off by " + aimError + " deg");
        check(endError(run, park) < 1.5, "parks in the same slot, off by " + endError(run, park) + " in");
        check(Math.abs(Math.toDegrees(run.robot.h)) < 3, "and square again, off by " + Math.toDegrees(run.robot.h) + " deg");
        check(run.finishedAt < PollenShootTuning.DEADLINE - 1, "finishes before the 27 s stop: " + run.finishedAt);
        Run timer = simulate(angled.sequence(), startCrooked(2));
        check(timer.result.startsWith("DONE") && timer.finishedAt < angled.worstCaseSeconds() + 1.5,
                "no speed reading: still finishes, about when the budget says: " + timer.finishedAt + " vs " + angled.worstCaseSeconds());
        System.out.printf(java.util.Locale.US, "  Angled Primary Shooter: feeds at %.2f s from %.2f in off the spot, %.2f deg off aim; "
                        + "finish %.2f s (planned worst case %.2f), park error %.2f in%n",
                run.feedAt, spotError, aimError, run.finishedAt, angled.worstCaseSeconds(), endError(run, park));

        // Without a shooter (or with the shot skipped) there is nothing to aim: no turns.
        Run noShooter = simulate(angled.sequence(false), new SimRobot());
        check(noShooter.steps.toString().equals("[SQUARE TO WALL, ZERO ODOMETRY, LEG 1 OFF WALL, LEG 2 TO SHOOTING SPOT, LEG 3 TO PARK]"),
                "no shooter: the three legs, no turns: " + noShooter.steps);
        check(endError(noShooter, park) < 1.5, "no shooter: still parks in the slot");
        PollenShootTuning noShot = new PollenShootTuning(null, false, true, true);
        setRow(noShot, FEED_SECONDS, 0);
        check(noShot.aimTurnSeconds() == 0 && !simulate(noShot.sequence(), new SimRobot()).steps.contains("TURN TO AIM"),
                "feed 0 s skips the shot and both turns");

        // The menu. An edit that would swing a corner into the wall is refused.
        PollenShootTuning tight = new PollenShootTuning(null, false, true, true);
        while (tight.selected() != LEG1_CM) tap(tight, false, true, false, false);
        for (int i = 0; i < 10; i++) tap(tight, false, false, true, false);
        check(tight.roomToTurn() && tight.legCm(1) > 0 && tight.notice().contains("hit the wall"),
                "Leg 1 cannot be shortened until the turn would hit the wall: " + tight.legCm(1) + " cm, " + tight.notice());
        setRow(tight, AIM_ANGLE, -30);
        check(tight.route() == PollenShootTuning.Route.EDITED, "changing the aim angle shows EDITED LEGS");
        PollenShootTuning next = new PollenShootTuning(tight.snapshot(), false, true, true);
        check(next.aimDegrees() == -30 && next.route() == PollenShootTuning.Route.EDITED, "the aim angle is remembered");
        next.update(false, false, false, false, true);
        check(next.route() == PollenShootTuning.Route.ANGLED_PARK && Math.abs(next.aimDegrees() + 36) < 0.5,
                "Triangle restores the angled defaults: " + next.aimDegrees());
        try {
            new PollenShootTuning(null, true, true, true);
            check(false, "angled + supporting must be rejected");
        } catch (IllegalArgumentException expected) { checks++; }
        try {
            new PollenShootTuning(null, false, false, true);
            check(false, "angled + timed must be rejected");
        } catch (IllegalArgumentException expected) { checks++; }

        // A robot that cannot turn at the spot never shoots sideways into the field.
        SimRobot jammed = new SimRobot();
        jammed.cannotTurnInPlace = true;
        Run stopped = simulate(new PollenShootTuning(null, false, true, true).sequence(), jammed);
        check(stopped.result.equals("TURN TO AIM NOT TURNING - STOPPED") && Double.isNaN(stopped.feedAt),
                "cannot turn at the spot: stops, and does not shoot: " + stopped.result);
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
        feedWhenUpToSpeed();
        turns();
        angledShot();
        speedRuns();
        System.out.println("PollenOdometryTest: " + checks + " checks passed (simulated robot, not robot validation)");
    }

    static void controller() {
        PollenOdometryDrive.Command far = PollenOdometryDrive.command(48, 0, 0.4, 0, 0, 0);
        check(Math.abs(far.forward - 0.4) < 1e-9 && far.strafe == 0 && far.turn == 0, "far ahead: full forward power");
        check(far.inchesToGo == 48 && far.driving(), "far ahead: 48 in to go, and driving");
        PollenOdometryDrive.Command left = PollenOdometryDrive.command(0, 48, 0.4, 0, 0, 0);
        check(Math.abs(left.strafe + 0.4) < 1e-9, "target to the left: strafe is negative (strafe + is right)");
        // Robot turned 90 deg counterclockwise: a field-forward target is to its RIGHT.
        PollenOdometryDrive.Command turned = PollenOdometryDrive.command(48, 0, 0.4, 0, 0, Math.PI / 2);
        check(Math.abs(turned.forward) < 1e-9 && turned.strafe > 0.39, "rotated robot strafes right toward field-forward target");
        check(turned.turn == PollenOdometryDrive.MAX_TURN, "rotated CCW: turns clockwise (capped)");
        PollenOdometryDrive.Command ccw = PollenOdometryDrive.command(48, 0, 0.4, 0, 0, Math.toRadians(5));
        check(ccw.turn > 0 && ccw.turn < 0.1, "5 deg CCW: small clockwise correction");
        PollenOdometryDrive.Command near = PollenOdometryDrive.command(1, 0, 0.4, 0, 0, 0);
        check(Math.abs(near.forward - 0.2) < 1e-9, "1 in out: slowed to 0.2");
        PollenOdometryDrive.Command nearer = PollenOdometryDrive.command(0.6, 0, 0.4, 0, 0, 0);
        check(Math.abs(nearer.forward - Math.max(PollenOdometryDrive.MIN_POWER, 0.2 * Math.sqrt(0.6))) < 1e-9,
                "just outside tolerance: never below the minimum power");
        PollenOdometryDrive.Command done = PollenOdometryDrive.command(0.3, 0.2, 0.4, 0, 0, 0.1);
        check(done.forward == 0 && done.strafe == 0 && done.turn == 0 && !done.driving(), "inside tolerance: all zero");
        check(Math.abs(done.inchesToGo - Math.hypot(0.3, 0.2)) < 1e-9, "inside tolerance: still reports the distance left");
        // Moving at 40 in/s, 3 in out: it will coast there within the look-ahead, so power is cut.
        PollenOdometryDrive.Command coasting = PollenOdometryDrive.command(3, 0, 0.4, 0, 0, 0, 40, 0);
        check(!coasting.driving() && coasting.inchesToGo == 3, "coasting up to the end point: no power, 3 in to go");
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
