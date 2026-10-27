import org.firstinspires.ftc.teamcode.pollen.*;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Irregular loop timing: no leg may be skipped or shortened by a slow loop, and a long
 * gap while the drive is moving stops everything. Time between two loops is credited to
 * the command the earlier loop sent, because the motors hold their last power. */
public final class PollenLoopTimingTest {
    private static int checks;
    private static void check(boolean ok, String message) {
        checks++;
        if (!ok) throw new AssertionError(message);
    }

    /** Result of one simulated run: seconds each drive phase was commanded, in order. */
    private static final class Run {
        final Map<String, Double> driven = new LinkedHashMap<>();
        final List<String> order = new ArrayList<>();
        PollenSequence.Output last;
        double end;
    }

    /** Loop at {@code period(i)} seconds; after any loop whose phase starts with "NEXT",
     * insert {@code transitionGap} extra seconds before the following loop. */
    private static Run simulate(PollenSequence s, java.util.function.IntToDoubleFunction period,
            double transitionGap) {
        Run r = new Run();
        s.start(0);
        double now = 0, previous = 0;
        PollenSequence.Output prevOut = null;
        for (int i = 0; i < 20000; i++) {
            PollenSequence.Output out = s.tick(now);
            if (prevOut != null && (prevOut.forward != 0 || prevOut.strafe != 0)) {
                r.driven.merge(prevOut.phase, now - previous, Double::sum);
            }
            if ((out.forward != 0 || out.strafe != 0) && !r.order.contains(out.phase)) r.order.add(out.phase);
            if (out.finished) { r.last = out; r.end = now; return r; }
            previous = now;
            prevOut = out;
            now += period.applyAsDouble(i) + (out.phase.startsWith("NEXT") ? transitionGap : 0);
        }
        throw new AssertionError("sequence never finished");
    }

    private static void fullLegs(Run r, PollenShootTuning t, double maxLoop, String what) {
        int n = 0;
        for (int leg = 1; leg <= PollenShootTuning.LEGS; leg++) {
            if (t.legCm(leg) <= 0) continue;
            String name = t.legName(leg);
            Double got = r.driven.get(name);
            double planned = t.legMove(leg).seconds;
            check(got != null, what + ": " + name + " was skipped");
            check(got >= planned - 1e-9, what + ": " + name + " ran " + got + " s of " + planned);
            check(got <= planned + maxLoop + 1e-9, what + ": " + name + " overran: " + got + " s of " + planned);
            check(r.order.get(n++).equals(name), what + ": legs out of order " + r.order);
        }
        // Every leg above ran in full. The 27 s cutoff may still trim the final 0.5 s hold,
        // when the robot is already parked; anything else is a failure.
        check(r.last.phase.startsWith("DONE") || r.last.phase.startsWith("TIME LIMIT"), what + ": ended " + r.last.phase);
        check(r.end < PollenShootTuning.DEADLINE + maxLoop + 1e-9, what + ": ran past the cutoff loop, to " + r.end);
    }

    public static void main(String[] args) {
        PollenShootTuning c = new PollenShootTuning(null, true);
        PollenShootTuning b = new PollenShootTuning(null);
        b.update(false, false, false, false, false, true);   // Square: B field preset
        b.update(false, false, false, false, false, false);

        // Reviewer's case: a 350 ms pause right after each step change, at a steady 20 ms loop.
        fullLegs(simulate(c.sequence(), i -> .02, .35), c, .02, "C, 350 ms gap after each step change");
        fullLegs(simulate(b.sequence(), i -> .02, .35), b, .02, "B Square, 350 ms gap after each step change");
        // Same with the shooter unavailable (C keeps its feed slot; B departs at once).
        fullLegs(simulate(c.sequence(false), i -> .02, .35), c, .02, "C, no shooter");
        fullLegs(simulate(b.sequence(false), i -> .02, .35), b, .02, "B Square, no shooter");

        // Margin at a steady 20 ms loop: C's last leg ends with about 1.5 s to spare before 27 s.
        Run steady = simulate(c.sequence(), i -> .02, 0);
        check(steady.last.phase.startsWith("DONE") && steady.end < 26.1, "C finishes at " + steady.end);

        // Irregular loops: a repeating 5-240 ms pattern, every gap under the 250 ms stall limit.
        double[] jitter = {.005, .03, .012, .24, .018, .09, .007, .15, .021, .06, .2, .011};
        fullLegs(simulate(c.sequence(), i -> jitter[i % jitter.length], .3), c, .24, "C, irregular loop");
        fullLegs(simulate(b.sequence(), i -> jitter[i % jitter.length], .3), b, .24, "B Square, irregular loop");
        PollenShootTuning tested = new PollenShootTuning(null);  // B Triangle: the tested baseline
        fullLegs(simulate(tested.sequence(), i -> jitter[i % jitter.length], .3), tested, .24, "B Triangle, irregular loop");

        // A pause while the drive is moving: 250 ms is tolerated, anything longer stops everything.
        PollenSequence s = c.sequence(false); s.start(0);
        double now = 0; PollenSequence.Output o = null;
        for (; now < 30; now += .02) { o = s.tick(now); if (o.forward != 0) break; }
        check(o.phase.equals("LEG 1 CLEAR FLOWER"), "reached leg 1");
        o = s.tick(now + .25);
        check(!o.finished && o.forward != 0, "a 250 ms pause while driving is tolerated");
        o = s.tick(now + .25 + .2501);
        check(o.finished && o.phase.startsWith("LOOP STALL"), "a longer pause while driving stops: " + o.phase);
        check(o.forward == 0 && o.strafe == 0 && o.launcherVelocity == 0 && o.feederPower == 0, "stall stop zeroes outputs");
        check(s.warning().contains("paused"), "stall reason reported");
        o = s.tick(now + 2);
        check(o.finished && o.forward == 0, "stall stop is latched");

        // Long pauses while stationary (delay, spinup, feeding, between steps) never stop the run.
        s = c.sequence(); s.start(0);
        check(!s.tick(5).finished, "pause during the start delay");
        check(!s.tick(11.5).finished, "pause during prespin");
        s.tick(14.8);
        check(!s.tick(16).finished, "pause while feeding");

        // Option A uses the same sequencer: a stall during its forward drive also stops it.
        PollenSequence park = PollenSequence.park(PollenConfig.parkRoute(2, .2), 27, 0);
        park.start(0);
        check(park.tick(0).forward == .2, "A drives");
        check(park.tick(.4).phase.startsWith("LOOP STALL"), "A stops on a stall while driving");

        System.out.println("PollenLoopTimingTest: " + checks + " checks passed");
    }
}
