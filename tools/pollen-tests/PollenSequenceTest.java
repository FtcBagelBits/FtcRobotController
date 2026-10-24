import org.firstinspires.ftc.teamcode.pollen.PollenConfig;
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
    private static PollenSequence.Output run(PollenSequence s, double from, double to, double velocity) {
        PollenSequence.Output out = null;
        for (double t = from; t < to - 1e-9; t += 0.02) out = s.tick(t, velocity, false);
        return out;
    }
    public static void main(String[] args) {
        PollenSequence park = sequence(new PollenSequence.Step[]{
                PollenSequence.Step.drive("MOVE", .75, .2, 0, 0),
                PollenSequence.Step.drive("HOLD", .5, 0, 0, 0)});
        zero(park.tick(0, 0, false)); // No action before Start.
        park.start(0);
        check(park.tick(0, 0, false).forward == 0.2, "Park movement");
        check(run(park, 0.02, 0.745, 0).forward == 0.2, "Park moves for its whole step");
        zero(park.tick(0.75, 0, false));
        PollenSequence.Output done = run(park, 0.77, 1.28, 0);
        check(done.finished && done.phase.startsWith("DONE"), "Park completion");
        zero(done);
        zero(park.tick(2, 0, false));

        // Fixed timing: launcher alone for 3 s, then feed 2 s. Measured speed never gates the feed.
        PollenSequence shot = shot(); shot.start(0);
        PollenSequence.Output spin = shot.tick(0, 0, false);
        check(spin.launcherVelocity == 1250 && spin.feederPower == 0, "Launcher alone during spin-up");
        check(run(shot, .02, 2.995, 0).feederPower == 0, "No feed during the 3 s spin-up");
        PollenSequence.Output feeding = shot.tick(3.0, 0, false);
        check(feeding.feederPower == 1 && feeding.launcherVelocity == 1250, "Feed starts at 3 s even with zero measured speed");
        feeding = shot.tick(3.5, 900, false);
        check(feeding.feederPower == 1, "A speed dip does not pause the feed");
        check(Math.abs(feeding.activeFeedSeconds - .5) < 1e-9, "Feed time counted from feed start");
        check(shot.tick(4.99, 1250, false).feederPower == 1, "Full 2 s feed after spin-up");
        zero(shot.tick(5.0, 1250, false));
        check(shot.tick(5.02, 0, false).forward == .2, "Park after the feed completes");
        check(run(shot, 5.04, 6.015, 0).forward == .2, "Park leg runs a full second from its first loop");
        PollenSequence.Output shotDone = shot.tick(6.03, 0, false);
        check(shotDone.finished && shotDone.phase.startsWith("DONE"), "Shot route completes");

        PollenSequence delayed = new PollenSequence(new PollenSequence.Step[]{
                PollenSequence.Step.drive("MOVE", 1, .2, 0, 0)}, 1250, 3, 27, 2);
        delayed.start(10);
        zero(delayed.tick(10, 0, false));
        zero(delayed.tick(11.99, 0, false));
        check(delayed.tick(12, 0, false).forward == .2, "Delay completes before movement");
        check(run(delayed, 12.02, 12.995, 0).forward == .2, "Delay preserves move duration");
        PollenSequence.Output delayedDone = delayed.tick(13.01, 0, false);
        check(delayedDone.finished && delayedDone.phase.startsWith("DONE"), "Delayed move completes");
        delayed.start(20);
        zero(delayed.tick(21, 0, true));
        check(delayed.tick(22, 0, false).finished, "Stop during delay is latched");

        // A launcher that never reports speed (e.g. unplugged encoder) still feeds on schedule.
        PollenSequence blind = shot(); blind.start(0);
        boolean fedBlind = false;
        for (int i = 0; i <= 250; i++) fedBlind |= blind.tick(i * .02, 0, false).feederPower > 0;
        check(fedBlind && blind.warning().isEmpty(), "Zero measured speed: feeds on schedule, no warning");
        check(blind.tick(5.02, 0, false).forward == .2, "then parks");
        PollenSequence noSpin = new PollenSequence(new PollenSequence.Step[]{PollenSequence.Step.continuousShoot(1)},
                1250, 0, 27, 0);
        noSpin.start(0);
        check(noSpin.tick(0, 0, false).feederPower == 1, "Zero spin-up feeds at once");
        PollenSequence badSpeed = shot(); badSpeed.start(0);
        check(badSpeed.tick(0, Double.NaN, false).finished, "Invalid launcher data still stops everything");

        // Stop before start, during driving, spinup, feeding, and parking must latch zero.
        for (double stopAt : new double[]{0, 1.5, 3.5, 5.5, 6.5}) {
            PollenSequence s = shot(); s.start(0);
            for (double t = 0; t < stopAt; t += 0.02) s.tick(t, 1250, false);
            zero(s.tick(stopAt, 1250, true));
            PollenSequence.Output stopped = s.tick(stopAt + 0.1, 1250, false);
            zero(stopped); check(stopped.finished, "Stop latched");
        }
        PollenSequence stoppedBeforeStart = shot();
        zero(stoppedBeforeStart.tick(0, 0, true));
        check(stoppedBeforeStart.tick(1, 0, false).finished, "Stop before start latched");

        PollenSequence deadline = shot(); deadline.start(0);
        zero(deadline.tick(27, 1250, false));
        check(deadline.tick(28, 1250, false).finished, "Hard deadline latched");
        PollenSequence clock = shot(); clock.start(2);
        check(clock.tick(1, 1250, false).finished, "Bad clock stops");

        PollenSequence diagonal = sequence(new PollenSequence.Step[]{
                PollenSequence.Step.drive("diagonal", 1, 1, 1, 1)});
        diagonal.start(0);
        double[] wheels = PollenSequence.wheelPowers(diagonal.tick(0, 0, false));
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
                    PollenSequence.Output out = s.tick(t, failure ? 0 : 1250, false);
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
        System.out.println("PASS: " + checks + " checks (pure Java sequence; not Android/hardware validation)");
    }
}
