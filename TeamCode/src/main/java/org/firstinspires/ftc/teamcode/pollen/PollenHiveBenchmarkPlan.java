package org.firstinspires.ftc.teamcode.pollen;

/** Timed camera-only experiment. Humans label physical truth, never the detector. */
public final class PollenHiveBenchmarkPlan {
    public enum Phase { WAIT, SWITCH, SETTLE, RECORD, DONE, ERROR }
    public Phase phase = Phase.WAIT;
    public final int first, last, repeats;
    public final double seconds;
    public int block, slot;
    public double since;
    public String error = "";
    public static final double SETTLE_SECONDS = 2;
    public PollenHiveBenchmarkPlan(int first, int last, int repeats, double seconds) {
        if (first < 0 || last > 9 || first > last || repeats < 1 || repeats > 5
                || !Double.isFinite(seconds) || seconds < 5 || seconds > 60)
            throw new IllegalArgumentException("Invalid benchmark plan");
        this.first = first; this.last = last; this.repeats = repeats; this.seconds = seconds;
    }
    public String expected() { return block % 2 == 0 ? "UP" : "DOWN"; }
    public int pass() { return block / 2 + 1; }
    public int trial() { return block * (last - first + 1) + slot; }
    /** Reverse each pass, keeping UP and DOWN paired in the same order. */
    public int pipeline() { return pass() % 2 == 1 ? first + slot : last - slot; }
    public void confirm(String physical, double now) {
        if (phase == Phase.WAIT && expected().equals(physical)) { slot = 0; phase = Phase.SWITCH; since = now; }
    }
    public void switched(boolean ok, double now) {
        if (phase != Phase.SWITCH) return;
        if (!ok) { fail("Pipeline switch rejected"); return; }
        phase = Phase.SETTLE; since = now;
    }
    public void tick(double now) {
        if (phase == Phase.SETTLE && now - since >= SETTLE_SECONDS) { phase = Phase.RECORD; since = now; }
        else if (phase == Phase.RECORD && now - since >= seconds) {
            if (++slot <= last - first) phase = Phase.SWITCH;
            else if (++block >= repeats * 2) phase = Phase.DONE;
            else phase = Phase.WAIT;
            since = now;
        }
    }
    public void fail(String why) { phase = Phase.ERROR; error = why; }
}
