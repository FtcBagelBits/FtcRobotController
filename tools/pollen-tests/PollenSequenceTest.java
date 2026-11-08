import org.firstinspires.ftc.teamcode.pollen.PollenAutoLog;
import org.firstinspires.ftc.teamcode.pollen.PollenConfig;
import org.firstinspires.ftc.teamcode.pollen.PollenOdometryDrive;
import org.firstinspires.ftc.teamcode.pollen.PollenSequence;
import org.firstinspires.ftc.teamcode.pollen.PollenShootTuning;

/** Run with tools/test-pollen-sequence.sh. No Android SDK or simulated FTC classes. */
public final class PollenSequenceTest {
    private static int checks;
    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static void zero(PollenSequence.Output out) {
        check(out.forward == 0 && out.strafe == 0 && out.turn == 0
                && out.launcherVelocity == 0 && out.feederPower == 0, "Nonzero stopped output");
    }
    private static PollenSequence sequence(PollenSequence.Step[] route) {
        return new PollenSequence(route, 1250, 3, 27, 0); // 3 s spin-up, then feed
    }
    private static PollenSequence shot() {
        return sequence(new PollenSequence.Step[]{PollenSequence.Step.continuousShoot(2),
                PollenSequence.Step.drive("PARK", 1, 0.2, 0, 0)});
    }
    /** Tick every 20 ms from {@code from} up to (not including) {@code to}; returns the last output. */
    private static PollenSequence.Output run(PollenSequence s, double from, double to) {
        PollenSequence.Output out = null;
        for (double t = from; t < to - 1e-9; t += 0.02) out = s.tick(t);
        return out;
    }
    /** Odometry move steps fed a scripted pose: pose[0..2] = x, y, heading as functions of time. */
    interface Pose { double[] at(double t); }
    private static PollenSequence.Output runMove(PollenSequence s, double from, double to, Pose pose,
                                                 java.util.List<String> phases) {
        PollenSequence.Output out = null;
        for (double t = from; t < to - 1e-9; t += 0.02) {
            double[] p = pose.at(t);
            out = s.tick(t, p[0], p[1], p[2]);
            phases.add(out.phase);
            if (out.finished) break;
        }
        return out;
    }
    private static void odometryFailures() {
        java.util.List<String> phases = new java.util.ArrayList<>();
        // 1. Frozen but finite pose (unplugged pods, frozen reading, or a blocked robot): stop
        //    after the stuck time instead of driving blind for the leg's whole timeout.
        PollenSequence frozen = PollenSequence.park(new PollenSequence.Step[]{
                PollenSequence.Step.moveTo("CLEAR", 20, 0, 0.6, 1, 3),
                PollenSequence.Step.moveTo("SIDE", 20, 20, 0.6, 1, 3)}, 27, 0);
        frozen.start(0);
        PollenSequence.Output out = runMove(frozen, 0, 5, t -> new double[]{0, 0, 0}, phases);
        check(out.finished && out.phase.equals("CLEAR NOT MOVING - STOPPED"), "Frozen pose stops: " + out.phase);
        check(!phases.contains("SIDE"), "Frozen pose never starts the next leg");
        int driven = 0;
        for (String p : phases) if (p.equals("CLEAR")) driven++;
        check(driven * 0.02 <= PollenOdometryDrive.NO_PROGRESS_SECONDS + 0.05, "Frozen pose drives at most the stuck time: " + driven * 0.02);
        zero(frozen.tick(5.02, 0, 0, 0));
        check(frozen.warning().contains("short"), "Stuck warning says how far short: " + frozen.warning());

        // 2. Slow progress (so not "stuck") that runs out of time far from the end: STOP, do
        //    not cut across to the next leg's end point.
        phases.clear();
        PollenSequence slow = PollenSequence.park(new PollenSequence.Step[]{
                PollenSequence.Step.moveTo("CLEAR", 10, 0, 0.6, 0.5, 1),
                PollenSequence.Step.moveTo("SIDE", 10, 20, 0.6, 1, 3)}, 27, 0);
        slow.start(0);
        out = runMove(slow, 0, 3, t -> new double[]{t, 0, 0}, phases); // 1 in/s
        check(out.phase.equals("CLEAR TIMED OUT - STOPPED"), "Far timeout stops the route: " + out.phase);
        check(!phases.contains("SIDE"), "Far timeout never starts the sideways leg: " + phases);
        check(slow.warning().contains("ran out of time"), "Timeout warning: " + slow.warning());

        // 3. Timeout while almost there (within NEAR_ENOUGH_IN): carry on to the next leg.
        phases.clear();
        PollenSequence near = PollenSequence.park(new PollenSequence.Step[]{
                PollenSequence.Step.moveTo("CLEAR", 10, 0, 0.6, 0.5, 1),
                PollenSequence.Step.drive("HOLD", 0.5, 0, 0, 0)}, 27, 0);
        near.start(0);
        out = runMove(near, 0, 3, t -> new double[]{9, 0, 0}, phases); // 1 in short, not settling in
        check(out.phase.startsWith("DONE"), "Near-enough timeout carries on: " + out.phase);
        check(phases.contains("HOLD"), "Near-enough timeout runs the next step");
        check(near.warning().contains("close enough"), "Near-enough warning: " + near.warning());

        // 4. A final leg that runs out of time far away is not reported as DONE.
        PollenSequence last = PollenSequence.park(new PollenSequence.Step[]{
                PollenSequence.Step.moveTo("PARK", 10, 0, 0.6, 0.5, 1)}, 27, 0);
        last.start(0);
        out = runMove(last, 0, 3, t -> new double[]{t, 0, 0}, new java.util.ArrayList<String>());
        check(out.phase.equals("PARK TIMED OUT - STOPPED") && last.result().equals(out.phase), "Final-leg timeout is not DONE: " + out.phase);

        // 5. A robot that is standing still ON its end point is fine: unchanged numbers alone are not a fault.
        PollenSequence still = PollenSequence.park(new PollenSequence.Step[]{
                PollenSequence.Step.moveTo("PARK", 10, 0, 0.6, 0.5, 2)}, 27, 0);
        still.start(0);
        out = runMove(still, 0, 3, t -> new double[]{10, 0, 0}, new java.util.ArrayList<String>());
        check(out.phase.startsWith("DONE") && still.warning().isEmpty(), "Stationary at the end point is DONE: " + out.phase);

        // 6. Pinpoint statuses that block a start (hardware faults) vs. ones that do not.
        check(PollenOdometryDrive.pinpointFault("FAULT_X_POD_NOT_DETECTED"), "X pod missing is a fault");
        check(PollenOdometryDrive.pinpointFault("FAULT_IMU_RUNAWAY"), "IMU runaway is a fault");
        check(!PollenOdometryDrive.pinpointFault("FAULT_BAD_READ"), "A single bad read is not");
        check(!PollenOdometryDrive.pinpointFault("READY") && !PollenOdometryDrive.pinpointFault("CALIBRATING")
                && !PollenOdometryDrive.pinpointFault(null), "READY / CALIBRATING / unknown are not");
    }
    /** The robot log: every line the run needs to be explained later, and never an exception. */
    private static void autoLog() {
        final java.util.List<String> lines = new java.util.ArrayList<>();
        PollenAutoLog log = new PollenAutoLog("Odo ROBOT Shoot + Park", lines::add);
        log.initRoute("FULL_PARK");
        log.initRoute("FULL_PARK");
        log.initRoute("LEAVE_ONLY");
        check(lines.size() == 2 && lines.get(1).equals("INIT Odo ROBOT Shoot + Park route=LEAVE_ONLY"),
                "INIT route logged once per change: " + lines);
        log.status("READY"); log.status("READY");
        check(lines.size() == 3 && lines.get(2).equals("PINPOINT status=READY"), "Pinpoint status logged on change");
        log.start("route=LEAVE_ONLY LEG3_CM=60.96", "shooter=ok", () -> 12.7);
        check(lines.get(3).startsWith("START Odo ROBOT Shoot + Park build=" + PollenConfig.BUILD + " route=LEAVE_ONLY")
                && lines.get(3).endsWith("battery=12.70V"), "START line: " + lines.get(3));

        PollenSequence frozen = PollenSequence.park(new PollenSequence.Step[]{
                PollenSequence.Step.moveTo("CLEAR", 20, 0, 0.6, 1, 3)}, 27, 0);
        frozen.start(0);
        for (double t = 0; t < 2; t += 0.02) {
            PollenSequence.Output out = frozen.tick(t, 0, 0, 0);
            log.loop(t, out, frozen.warning(), 0, 0, 0, PollenSequence.wheelPowers(out), 0, () -> 12.5);
        }
        log.stop(2);
        String all = String.join("\n", lines);
        check(all.contains("PHASE CLEAR | x=0.0 y=0.0 h=0.0 battery=12.50V"), "phase line with pose: " + all);
        check(all.contains("WARNING CLEAR: wheels driven"), "warning line");
        check(all.contains("RESULT CLEAR NOT MOVING - STOPPED"), "result line");
        check(lines.get(lines.size() - 1).contains("STOP (OpMode ended; last phase: CLEAR NOT MOVING - STOPPED)"), "stop line");
        int samples = 0, results = 0;
        for (String l : lines) { if (l.contains(" togo=")) samples++; if (l.contains("RESULT")) results++; }
        check(samples >= 3 && samples <= 5, "about one sample per 0.25 s while driving: " + samples);
        check(results == 1, "one RESULT line");

        // Logging never throws: a broken sink, a broken battery read, NaN pose (timed modes).
        PollenAutoLog broken = new PollenAutoLog("X", text -> { throw new IllegalStateException("disk full"); });
        PollenSequence s = shot(); s.start(0);
        PollenSequence.Output out = s.tick(0);
        broken.loop(0, out, "", Double.NaN, Double.NaN, Double.NaN, PollenSequence.wheelPowers(out), 0, () -> 1 / 0);
        broken.start("x", "y", null); broken.stop(1); broken.initRoute("R"); broken.status("S");
        lines.clear();
        PollenAutoLog timed = new PollenAutoLog("Pollen ROBOT Shoot + Park", lines::add);
        timed.loop(0, out, "", Double.NaN, Double.NaN, Double.NaN, PollenSequence.wheelPowers(out), 0,
                () -> { throw new IllegalStateException("no hub"); });
        check(lines.get(0).contains("x=NaN") && lines.get(0).contains("battery=?"), "NaN pose and failed battery read still log: " + lines);
    }
    public static void main(String[] args) {
        PollenSequence park = sequence(new PollenSequence.Step[]{
                PollenSequence.Step.drive("MOVE", .75, .2, 0, 0),
                PollenSequence.Step.drive("HOLD", .5, 0, 0, 0)});
        zero(park.tick(0)); // No action before Start.
        park.start(0);
        check(park.tick(0).forward == 0.2, "Park movement");
        check(run(park, 0.02, 0.745).forward == 0.2, "Park moves for its whole step");
        zero(park.tick(0.75));
        PollenSequence.Output done = run(park, 0.77, 1.28);
        check(done.finished && done.phase.startsWith("DONE"), "Park completion");
        zero(done);
        zero(park.tick(2));

        // Fixed timing: launcher alone for 3 s, then feed 2 s. Measured speed never gates the feed.
        PollenSequence shot = shot(); shot.start(0);
        PollenSequence.Output spin = shot.tick(0);
        check(spin.launcherVelocity == 1250 && spin.feederPower == 0, "Launcher alone during spin-up");
        check(run(shot, .02, 2.995).feederPower == 0, "No feed during the 3 s spin-up");
        PollenSequence.Output feeding = shot.tick(3.0);
        check(feeding.feederPower == 1 && feeding.launcherVelocity == 1250, "Feed starts at 3 s even with zero measured speed");
        feeding = shot.tick(3.5);
        check(feeding.feederPower == 1, "A speed dip does not pause the feed");
        check(Math.abs(feeding.activeFeedSeconds - .5) < 1e-9, "Feed time counted from feed start");
        check(shot.tick(4.99).feederPower == 1, "Full 2 s feed after spin-up");
        zero(shot.tick(5.0));
        check(shot.tick(5.02).forward == .2, "Park after the feed completes");
        check(run(shot, 5.04, 6.015).forward == .2, "Park leg runs a full second from its first loop");
        PollenSequence.Output shotDone = shot.tick(6.03);
        check(shotDone.finished && shotDone.phase.startsWith("DONE"), "Shot route completes");

        PollenSequence delayed = new PollenSequence(new PollenSequence.Step[]{
                PollenSequence.Step.drive("MOVE", 1, .2, 0, 0)}, 1250, 3, 27, 2);
        delayed.start(10);
        zero(delayed.tick(10));
        zero(delayed.tick(11.99));
        check(delayed.tick(12).forward == .2, "Delay completes before movement");
        check(run(delayed, 12.02, 12.995).forward == .2, "Delay preserves move duration");
        PollenSequence.Output delayedDone = delayed.tick(13.01);
        check(delayedDone.finished && delayedDone.phase.startsWith("DONE"), "Delayed move completes");
        delayed.start(20);
        zero(delayed.stop("STOPPED"));
        check(delayed.tick(22).finished, "Stop during delay is latched");

        // A launcher that never reports speed (e.g. unplugged encoder) still feeds on schedule.
        PollenSequence blind = shot(); blind.start(0);
        boolean fedBlind = false;
        for (int i = 0; i <= 250; i++) fedBlind |= blind.tick(i * .02).feederPower > 0;
        check(fedBlind && blind.warning().isEmpty(), "Zero measured speed: feeds on schedule, no warning");
        check(blind.tick(5.02).forward == .2, "then parks");
        PollenSequence noSpin = new PollenSequence(new PollenSequence.Step[]{PollenSequence.Step.continuousShoot(1)},
                1250, 0, 27, 0);
        noSpin.start(0);
        check(noSpin.tick(0).feederPower == 1, "Zero spin-up feeds at once");

        // Stop before start, during driving, spinup, feeding, and parking must latch zero.
        for (double stopAt : new double[]{0, 1.5, 3.5, 5.5, 6.5}) {
            PollenSequence s = shot(); s.start(0);
            for (double t = 0; t < stopAt; t += 0.02) s.tick(t);
            zero(s.stop("STOPPED"));
            PollenSequence.Output stopped = s.tick(stopAt + 0.1);
            zero(stopped); check(stopped.finished, "Stop latched");
        }
        PollenSequence stoppedBeforeStart = shot();
        zero(stoppedBeforeStart.stop("STOPPED"));
        check(stoppedBeforeStart.tick(1).finished, "Stop before start latched");

        PollenSequence deadline = shot(); deadline.start(0);
        zero(deadline.tick(27));
        check(deadline.tick(28).finished, "Hard deadline latched");
        PollenSequence clock = shot(); clock.start(2);
        check(clock.tick(1).finished, "Bad clock stops");

        PollenSequence diagonal = sequence(new PollenSequence.Step[]{
                PollenSequence.Step.drive("diagonal", 1, 1, 1, 1)});
        diagonal.start(0);
        double[] wheels = PollenSequence.wheelPowers(diagonal.tick(0));
        for (double power : wheels) check(Math.abs(power) <= 1, "Motor normalization");
        check(wheels[0] == 1 && wheels[1] < 0, "Mecanum signs");

        boolean rejected = false;
        try { sequence(new PollenSequence.Step[]{PollenSequence.Step.drive("too long", 27, .2, 0, 0)}); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Reject route beyond deadline");
        rejected = false;
        try { PollenSequence.Step.drive("NaN", 1, Double.NaN, 0, 0); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Reject NaN route");

        // End-to-end both real route definitions with successful / failed simulated shooter.
        for (boolean shoot : new boolean[]{false, true}) {
            for (boolean failure : new boolean[]{false, true}) {
                PollenSequence s = shoot ? new PollenShootTuning(null).sequence()
                        : PollenSequence.park(PollenConfig.parkRoute(),
                                PollenConfig.TOTAL_TIME_LIMIT, PollenConfig.START_DELAY_SECONDS);
                s.start(0); boolean finished = false; boolean fed = false;
                for (double t = 0; t < PollenConfig.TOTAL_TIME_LIMIT + 1; t += .02) {
                    PollenSequence.Output out = s.tick(t);
                    fed |= out.feederPower > 0;
                    if (out.finished) { zero(out); finished = true; break; }
                }
                check(finished, "Full route finishes");
                check(fed == shoot, "Feeds on schedule whatever the measured launcher speed");
            }
        }
        rejected = false;
        try { sequence(new PollenSequence.Step[]{PollenSequence.Step.continuousShoot(23)}); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Budget includes spinup plus continuous feed");
        rejected = false;
        try { PollenSequence.Step.continuousShoot(0); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Reject zero-length shot step");
        rejected = false;
        try { PollenSequence.Step.continuousShoot(Double.NaN); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Reject invalid feed duration");
        rejected = false;
        try { PollenSequence.park(new PollenSequence.Step[]{PollenSequence.Step.continuousShoot(1)}, 27, 0); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "Drive-only sequence rejects shooting steps");
        rejected = false;
        try { new PollenSequence(new PollenSequence.Step[]{PollenSequence.Step.continuousShoot(1)}, 0, 3, 27, 0); }
        catch (IllegalArgumentException expected) { rejected = true; }
        check(rejected, "A shooting step needs a launcher velocity");
        for (double delay : new double[]{-1, Double.NaN, 25}) {
            rejected = false;
            try { new PollenSequence(new PollenSequence.Step[]{PollenSequence.Step.drive("MOVE", 1, .2, 0, 0)},
                    1250, 3, 27, delay); }
            catch (IllegalArgumentException expected) { rejected = true; }
            check(rejected, "Reject invalid or over-budget delay");
        }
        odometryFailures();
        autoLog();
        System.out.println("PASS: " + checks + " checks (pure Java sequence; not Android/hardware validation)");
    }
}
