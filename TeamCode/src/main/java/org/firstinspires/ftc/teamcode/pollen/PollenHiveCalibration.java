package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;

/** One three-second capture. The operator supplies the PHYSICAL state, never inferred from height.
 * Checks use the same detector as auto, with the cell fixed before the check begins. */
public final class PollenHiveCalibration {
    public static final double SECONDS = 3, MIN_SPAN = 1;
    public static final int MIN_FRAMES = 20;
    public static final double MAX_TILT_SD_DEG = .6, MAX_HEIGHT_SD_IN = .6, HEIGHT_MATCH_IN = 1;
    public final boolean tilt;
    public final IsHiveReadyForShot.State expected;
    private final IsHiveReadyForShot.Mount mount;
    private IsHiveReadyForShot detector;
    private double lastStamp = Double.NaN, first = Double.NaN, last, currentTime, started = Double.NaN;
    public int frames, rejected, fresh, empty, falseUp, matching, evaluated, cluster;
    private final Stats values = new Stats(), sideways = new Stats();
    private boolean wrongTagState;

    public PollenHiveCalibration(boolean tilt, IsHiveReadyForShot.Mount mount,
                                 IsHiveReadyForShot.State expected, int cluster) {
        if (expected == IsHiveReadyForShot.State.UNKNOWN || (tilt && expected != IsHiveReadyForShot.State.UP))
            throw new IllegalArgumentException("Label the physical cell UP or DOWN");
        if (!tilt && IsHiveReadyForShot.clusterOf(cluster) != cluster)
            throw new IllegalArgumentException("Identify our cell before checking");
        this.tilt = tilt; this.mount = mount; this.expected = expected; this.cluster = cluster;
        if (!tilt) detector = new IsHiveReadyForShot(mount, cluster);
    }
    public void add(IsHiveReadyForShot.Frame f, double now) {
        if (!Double.isFinite(now)) return;
        currentTime = now;
        if (Double.isNaN(started)) started = now;
        if (!tilt) detector.update(f, now);
        if (f == null || !f.usable()) { rejected++; return; }
        if (f.stamp == lastStamp) return;
        if (Double.isFinite(lastStamp) && f.stamp < lastStamp) { rejected++; lastStamp = f.stamp; return; }
        lastStamp = f.stamp; fresh++;
        if (cluster < 0) cluster = IsHiveReadyForShot.nearestCluster(f, mount, IsHiveReadyForShot.ALL_CELLS);
        double side = 0; int n = 0;
        double[] samples = new double[4];
        for (IsHiveReadyForShot.Tag t : f.tags) {
            if (IsHiveReadyForShot.clusterOf(t.id) != cluster || Math.abs(mount.sidewaysIn(t)) > mount.oursMaxSidewaysIn) continue;
            double v = tilt ? IsHiveReadyForShot.solveTiltDeg(t, mount.upIn, mount.lensHeightIn) : mount.heightIn(t);
            // A frame counts only if ALL of its accepted cell tags have a usable estimate.
            if (!Double.isFinite(v) || (tilt && (v < -10 || v > 80)) || n == 4) { rejected++; return; }
            if (!tilt && mount.classify(v) != expected) wrongTagState = true;
            samples[n++] = v; side += t.x * IsHiveReadyForShot.METRES_TO_INCHES;
        }
        if (n == 0) { empty++; if (tilt) { rejected++; return; } }
        else {
            frames++;
            // Include per-tag variation; opposing bad tags must not cancel in a mean.
            for (int i = 0; i < n; i++) values.add(samples[i]);
            if (tilt && n == 4) sideways.add(-side / 4);
        }
        if (Double.isNaN(first)) first = now;
        last = now;
        if (!tilt) {
            IsHiveReadyForShot.State observed = detector.state(now);
            if (expected == IsHiveReadyForShot.State.DOWN && observed == IsHiveReadyForShot.State.UP) falseUp++;
            if (now - started >= .4) { evaluated++; if (observed == expected) matching++; }
        }
    }
    public boolean blockedOnly() { return !tilt && expected == IsHiveReadyForShot.State.DOWN && frames == 0 && empty >= MIN_FRAMES; }
    public String problem() {
        if (!tilt && !mount.problem().isEmpty()) return mount.problem();
        if (last - first < MIN_SPAN || !Double.isFinite(first)) return "Need at least 1 second of distinct usable frames";
        if (currentTime - last > IsHiveReadyForShot.MAX_AGE) return "No recent distinct camera frame; repeat";
        if (falseUp > 0) return "FALSE UP while physically DOWN";
        if (blockedOnly()) return rejected == 0 ? "" : "Camera errors during hidden-DOWN check; repeat";
        if (frames < MIN_FRAMES) return "Only " + frames + " accepted frames; need " + MIN_FRAMES;
        if (!(values.sd() <= (tilt ? MAX_TILT_SD_DEG : MAX_HEIGHT_SD_IN))) return "Readings too noisy; hold still and repeat";
        if (tilt) return candidateMount().problem();
        if (wrongTagState) return "One or more tags disagreed with the physical label; no PASS";
        if (Math.abs(heightIn() - (expected == IsHiveReadyForShot.State.UP ? mount.upIn : mount.downIn)) > HEIGHT_MATCH_IN)
            return "Height differs by more than 1 in; check measurements in INIT";
        if (evaluated < MIN_FRAMES || matching < .9 * evaluated || detector.state(last) != expected)
            return "Production detector did not hold the expected state; repeat";
        return "";
    }
    public double tiltDeg() { return values.mean(); }
    public double heightIn() { return values.mean(); }
    public double sidewaysCorrectionIn() { return sideways.n >= MIN_FRAMES / 2 ? sideways.mean() : Double.NaN; }
    /** Round BEFORE validation and preview, so saved values are exactly the values checked. */
    public IsHiveReadyForShot.Mount candidateMount() {
        double side = sidewaysCorrectionIn();
        return new IsHiveReadyForShot.Mount(mount.lensHeightIn, Math.round(tiltDeg() * 10) / 10.0,
                Double.isNaN(side) ? mount.sidewaysCorrectionIn : Math.round(side * 100) / 100.0,
                mount.upIn, mount.downIn, mount.bandIn, mount.oursMaxSidewaysIn);
    }
    public String[] report() {
        String p = problem();
        return new String[]{String.format(Locale.US,
                "%s physical %s; cell %d; accepted=%d rejected=%d empty=%d span=%.2fs falseUP=%d stable=%d/%d",
                tilt ? "TILT" : "CHECK", expected, cluster, frames, rejected, empty, last-first, falseUp, matching, evaluated),
                p.isEmpty() ? (tilt ? "CANDIDATE: verify UP, then DOWN before saving"
                    : blockedOnly() ? "BLOCKED ONLY: DOWN tags hidden; no positive DOWN detection" : "PASS: physical label and production detector agree") : "REJECTED: " + p,
                String.format(Locale.US, "%s=%.2f, tag sample SD=%.2f; sideways=%.2f", tilt ? "tilt deg" : "height in", values.mean(), values.sd(), sidewaysCorrectionIn())};
    }
    private static final class Stats {
        int n; double sum, sumSq;
        void add(double v) { n++; sum += v; sumSq += v * v; }
        double mean() { return n == 0 ? Double.NaN : sum / n; }
        double sd() { return n < 2 ? Double.NaN : Math.sqrt(Math.max(0, (sumSq - sum * sum / n) / (n - 1))); }
    }
}
