package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;

/** Pure Java math behind TEST Hive camera (NO MOTORS). One object = one 3-second capture.
 * TILT capture (Square): robot at the PRIMARY start, our cell UP and still. Every tag of the cell
 *   straight behind gives the tilt that puts it at HIVE_UP_TAG_HEIGHT_IN. Frames that show all four
 *   tags also give the sideways correction that puts the cell's center straight behind.
 * HEIGHT check (Circle): anywhere, either state. Averages the measured height of the cell behind,
 *   and suggests a new UP/DOWN height when it is more than HEIGHT_MATCH_IN off. */
public final class PollenHiveCalibration {
    public static final double SECONDS = 3.0;
    public static final int MIN_FRAMES = 20;
    /** Largest standard deviation accepted: bigger means something moved or the view is poor. */
    public static final double MAX_TILT_SD_DEG = 0.6, MAX_HEIGHT_SD_IN = 0.6;
    /** A height check within this of the PollenConfig number is a pass. */
    public static final double HEIGHT_MATCH_IN = 1.0;

    public final boolean tilt;
    private final IsHiveReadyForShot.Mount mount;
    private double lastStamp = Double.NaN;
    public int frames, cluster = -1;
    private final Stats values = new Stats(), sideways = new Stats();

    public PollenHiveCalibration(boolean tilt, IsHiveReadyForShot.Mount mount) { this.tilt = tilt; this.mount = mount; }

    public void add(IsHiveReadyForShot.Frame f) {
        if (f == null || !f.usable() || f.stamp == lastStamp) return;
        int c = IsHiveReadyForShot.nearestCluster(f, mount, IsHiveReadyForShot.ALL_CELLS);
        if (c < 0 || (cluster >= 0 && c != cluster)) return;
        lastStamp = f.stamp; cluster = c; frames++;
        double side = 0, heights = 0; int n = 0;
        for (IsHiveReadyForShot.Tag t : f.tags) {
            if (IsHiveReadyForShot.clusterOf(t.id) != c) continue;
            n++; side += t.x * IsHiveReadyForShot.METRES_TO_INCHES;
            if (tilt) values.add(IsHiveReadyForShot.solveTiltDeg(t, mount.upIn, mount.lensHeightIn));
            else heights += mount.heightIn(t);
        }
        if (!tilt) values.add(heights / n);
        if (tilt && n == 4) sideways.add(-side / 4); // correction that makes the 4-tag center read 0
    }

    /** Empty if this capture can be used, otherwise why not. */
    public String problem() {
        if (frames < MIN_FRAMES) return "Only " + frames + " frames of one cell; need " + MIN_FRAMES + ". Is the cell in view?";
        if (!(values.sd() <= (tilt ? MAX_TILT_SD_DEG : MAX_HEIGHT_SD_IN))) return "Readings too noisy (robot or hive moving?). Repeat.";
        if (!tilt && !mount.problem().isEmpty()) return mount.problem();
        if (tilt && !(Math.abs(values.mean()) <= 80)) return "Tilt result out of range; check lens height and tag height";
        return "";
    }
    public double tiltDeg() { return values.mean(); }
    public double heightIn() { return values.mean(); }
    /** Height check of a DOWN cell that is more than HEIGHT_MATCH_IN off: the new DOWN height, else NaN. */
    public double suggestedDownIn() {
        if (tilt || !problem().isEmpty() || mount.classify(heightIn()) != IsHiveReadyForShot.State.DOWN) return Double.NaN;
        return Math.abs(heightIn() - mount.downIn) > HEIGHT_MATCH_IN ? Math.round(heightIn() * 10) / 10.0 : Double.NaN;
    }
    /** NaN unless enough frames showed all four tags. */
    public double sidewaysCorrectionIn() { return sideways.n >= MIN_FRAMES / 2 ? sideways.mean() : Double.NaN; }

    /** Telemetry/log lines. For a tilt capture these are the lines to paste into PollenConfig. */
    public String[] report() {
        String p = problem();
        if (!p.isEmpty()) return new String[]{"REJECTED: " + p};
        if (tilt) {
            double s = sidewaysCorrectionIn();
            return new String[]{
                String.format(Locale.US, "Cell %s, %d frames, tilt s.d. %.2f deg", IsHiveReadyForShot.clusterName(cluster), frames, values.sd()),
                String.format(Locale.US, "Camera tilt up deg = %.1f", tiltDeg()),
                Double.isNaN(s) ? "Sideways: all 4 tags not seen; correction unchanged"
                        : String.format(Locale.US, "Camera sideways correction in = %.2f", s),
                "Cross saves these on the Hub (all hive modes use them at once)."};
        }
        IsHiveReadyForShot.State st = mount.classify(heightIn());
        double ref = st == IsHiveReadyForShot.State.DOWN ? mount.downIn : mount.upIn;
        String verdict = st == IsHiveReadyForShot.State.UNKNOWN ? "BETWEEN BANDS: not UP, not DOWN. Re-check lens height and tilt."
                : Math.abs(heightIn() - ref) <= HEIGHT_MATCH_IN ? "PASS (within " + HEIGHT_MATCH_IN + " in)"
                : st == IsHiveReadyForShot.State.DOWN
                    ? String.format(Locale.US, "OFF BY %.1f in. If repeatable: DOWN tag height in = %.1f (Cross saves it)", heightIn() - ref, heightIn())
                    : String.format(Locale.US, "OFF BY %.1f in. Re-check lens height, then Square again", heightIn() - ref);
        return new String[]{
            String.format(Locale.US, "Cell %s: %s, %.1f in (s.d. %.2f, %d frames); expected %.1f",
                    IsHiveReadyForShot.clusterName(cluster), st, heightIn(), values.sd(), frames, ref),
            verdict};
    }

    /** Running mean and standard deviation. */
    private static final class Stats {
        int n; double sum, sumSq;
        void add(double v) { if (!Double.isFinite(v)) return; n++; sum += v; sumSq += v * v; }
        double mean() { return n == 0 ? Double.NaN : sum / n; }
        double sd() { return n < 2 ? Double.NaN : Math.sqrt(Math.max(0, (sumSq - sum * sum / n) / (n - 1))); }
    }
}
