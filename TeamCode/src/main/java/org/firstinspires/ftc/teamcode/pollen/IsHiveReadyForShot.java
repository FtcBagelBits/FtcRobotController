package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;

/** Is OUR hive cell (the one straight behind the robot) UP, so a shot can score?
 *
 * How it works, in three steps:
 * 1. WHICH CELL. Every hive cell carries four AprilTags with its own IDs (game manual 9.6):
 *    30-33 red scoring-side, 34-37 red audience-side, 38-41 blue audience-side, 42-45 blue
 *    scoring-side. From the PRIMARY start our cell is 34 (red) or 42 (blue); from the SECONDARY
 *    start 30 (red) or 38 (blue). Side AUTO: our cell is whichever of the two is nearest straight
 *    behind the robot, locked the first time it is seen steadily after START. Side RED or BLUE:
 *    the cell is fixed from the start.
 * 2. HOW HIGH. The Limelight reports each tag's position from the lens in metres: x right, y down,
 *    z out of the lens. With the lens height and the camera's upward tilt (camera rows of
 *    PollenHiveSettings; tilt fitted with TEST Hive camera) that gives the tag's height above the tiles:
 *        height = lensHeight + z*sin(tilt) - y*cos(tilt)
 *    Yaw/position do not change this height. It assumes a level chassis, a rigid mount,
 *    negligible camera roll and correctly scaled AprilTag poses.
 * 3. UP OR DOWN. An UP cell's tags are about 49.7 in above the tiles and a DOWN cell's about
 *    35.6 in (field CAD). Inside the UP band = UP, inside the DOWN band = DOWN, anything else
 *    (tipping, wrong tag, not calibrated, no tags) = UNKNOWN. Only a steady, fresh UP allows a shot.
 * 4. MISSING IS NOT CONTRARY. Three kinds of frame are treated differently:
 *    - CONTRADICTION (our tags seen between the bands, tags that disagree, the other state):
 *      takes permission away on that frame and restarts confirmation.
 *    - MISS (a new frame without our cell's tags: a ball in flight, blur, glare): no evidence
 *      either way. Keeps the state and confirmation progress, but only until MISS_HOLD after the
 *      last frame that did show our tags. That is the same 0.2 s a frozen camera already gets.
 *    - CAMERA FAULT (no frame, invalid frame, clock or camera restart): forgets everything.
 *    Without this, one missed frame cancelled permission and the next four frames had to
 *    re-confirm, so 90% per-frame detection fed only about 40-60% of the time.
 *
 * Call update every loop and ask isHiveReadyForShot with the current time; never cache the answer.
 */
public final class IsHiveReadyForShot {
    public enum State { UP, DOWN, UNKNOWN }
    /** UP/DOWN expires this long after the last new camera frame (seconds). */
    public static final double MAX_AGE = 0.20;
    /** A frame without our tags keeps the state until this long after the last frame that showed
     * them (seconds). A contradicting frame still revokes at once. */
    public static final double MISS_HOLD = MAX_AGE;
    /** A state counts only after this many frames showing our tags agree, with no contradiction
     * in between and no gap longer than MISS_HOLD, over at least this long. */
    public static final int CONFIRM_FRAMES = 4;
    public static final double CONFIRM_SECONDS = 0.15;
    public static final double METRES_TO_INCHES = 1 / 0.0254;

    /** One AprilTag as the camera saw it: ID and position from the lens in metres. */
    public static final class Tag {
        public final int id;
        public final double x, y, z;
        public Tag(int id, double x, double y, double z) { this.id = id; this.x = x; this.y = y; this.z = z; }
        boolean usable() {
            return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(z) && z > 0.05 && z < 15;
        }
    }

    /** One camera frame: its timestamp and every hive tag it saw (may be none). */
    public static final class Frame {
        public final double stamp;
        public final Tag[] tags;
        public Frame(double stamp, Tag... tags) { this.stamp = stamp; this.tags = tags == null ? new Tag[0] : tags.clone(); }
        boolean usable() {
            if (!Double.isFinite(stamp) || stamp <= 0) return false;
            for (Tag t : tags) if (t == null || !t.usable()) return false;
            return true;
        }
    }

    /** Camera mount and hive heights. fromConfig() reads the calibrated numbers in PollenConfig. */
    public static final class Mount {
        public final double lensHeightIn, tiltUpDeg, sidewaysCorrectionIn, upIn, downIn, bandIn, oursMaxSidewaysIn;
        public Mount(double lensHeightIn, double tiltUpDeg, double sidewaysCorrectionIn, double upIn,
                     double downIn, double bandIn, double oursMaxSidewaysIn) {
            this.lensHeightIn = lensHeightIn; this.tiltUpDeg = tiltUpDeg;
            this.sidewaysCorrectionIn = sidewaysCorrectionIn; this.upIn = upIn; this.downIn = downIn;
            this.bandIn = bandIn; this.oursMaxSidewaysIn = oursMaxSidewaysIn;
        }
        /** From the camera rows of PollenHiveSettings (defaults plus any saved changes). */
        public static Mount from(PollenHiveSettings.Values camera) {
            return new Mount(camera.get("lensHeight"), camera.get("tilt"), camera.get("sideways"),
                    camera.get("upHeight"), camera.get("downHeight"), camera.get("band"), camera.get("oursMaxSideways"));
        }
        /** Same mount with a different tilt (TEST Hive camera shows a candidate before it is committed). */
        public Mount withTilt(double tilt) {
            return new Mount(lensHeightIn, tilt, sidewaysCorrectionIn, upIn, downIn, bandIn, oursMaxSidewaysIn);
        }
        /** Empty when the numbers can classify; otherwise why not. NaN tilt = never calibrated. */
        public String problem() {
            if (!Double.isFinite(tiltUpDeg)) return "Camera tilt not calibrated (TEST Hive camera, Square)";
            if (tiltUpDeg < -10 || tiltUpDeg > 80) return "Camera tilt outside -10..80 deg";
            if (!(lensHeightIn > 2 && lensHeightIn < 30)) return "Lens height outside 2..30 in";
            if (!(bandIn > 0.5) || !Double.isFinite(upIn) || !Double.isFinite(downIn)
                    || upIn - downIn < 2 * bandIn + 2) return "UP/DOWN bands too close or invalid";
            if (!Double.isFinite(sidewaysCorrectionIn) || Math.abs(sidewaysCorrectionIn) > 12
                    || !Double.isFinite(oursMaxSidewaysIn) || !(oursMaxSidewaysIn > 0)) return "Invalid sideways settings";
            return "";
        }
        /** Height of a tag above the tiles, inches. Needs a calibrated tilt. */
        public double heightIn(Tag t) {
            double a = Math.toRadians(tiltUpDeg);
            return lensHeightIn + (t.z * Math.sin(a) - t.y * Math.cos(a)) * METRES_TO_INCHES;
        }
        /** Sideways offset of a tag from straight behind the robot, inches (after the mount correction). */
        public double sidewaysIn(Tag t) { return t.x * METRES_TO_INCHES + sidewaysCorrectionIn; }
        public State classify(double heightIn) {
            if (Math.abs(heightIn - upIn) <= bandIn) return State.UP;
            if (Math.abs(heightIn - downIn) <= bandIn) return State.DOWN;
            return State.UNKNOWN;
        }
    }

    /** Camera tilt (deg) that puts this tag at knownHeightIn: solves
     * knownHeight - lens = z*sin(a) - y*cos(a) for a. NaN if impossible. */
    public static double solveTiltDeg(Tag t, double knownHeightIn, double lensHeightIn) {
        double r = Math.hypot(t.y, t.z), rise = (knownHeightIn - lensHeightIn) / METRES_TO_INCHES;
        if (!(r > 0) || Math.abs(rise) > r) return Double.NaN;
        // z*sin(a) - y*cos(a) = r*sin(a - b) with b = atan2(y, z).
        return Math.toDegrees(Math.atan2(t.y, t.z) + Math.asin(rise / r));
    }

    /** First ID of a hive cell's cluster (30, 34, 38 or 42), or -1 for a tag that isn't a hive tag. */
    public static int clusterOf(int id) { return id >= 30 && id <= 45 ? 30 + (id - 30) / 4 * 4 : -1; }
    public static String clusterName(int cluster) {
        switch (cluster) {
            case 30: return "RED scoring-side cell (tags 30-33)";
            case 34: return "RED audience-side cell (tags 34-37)";
            case 38: return "BLUE audience-side cell (tags 38-41)";
            case 42: return "BLUE scoring-side cell (tags 42-45)";
            default: return "none";
        }
    }
    public static final int[] ALL_CELLS = {30, 34, 38, 42};
    /** Which cells can be ours. side: 0 AUTO, 1 RED, 2 BLUE (PollenHiveSettings "side" row). */
    public static int[] allowedCells(boolean secondary, int side) {
        int red = secondary ? 30 : 34, blue = secondary ? 38 : 42;
        return side == 1 ? new int[]{red} : side == 2 ? new int[]{blue} : new int[]{red, blue};
    }
    private static boolean contains(int[] cells, int c) { for (int x : cells) if (x == c) return true; return false; }
    /** The allowed hive cell nearest straight behind the robot in this frame, or -1 if none is close
     * enough. A cell counts by its most central visible tag; the other hive's tags are about 16 in or
     * more to the side from our start spots, ours at most about 10 in. */
    public static int nearestCluster(Frame f, Mount m, int... allowed) {
        if (f == null || !f.usable()) return -1;
        if (allowed.length == 0) allowed = ALL_CELLS;
        int best = -1;
        double bestSide = Double.POSITIVE_INFINITY;
        for (Tag t : f.tags) {
            int c = clusterOf(t.id);
            double side = Math.abs(m.sidewaysIn(t));
            if (c >= 0 && contains(allowed, c) && side < bestSide) { bestSide = side; best = c; }
        }
        return bestSide <= m.oursMaxSidewaysIn ? best : -1;
    }

    private final Mount mount;
    private final int[] allowed;
    private int cluster = -1, candidateCluster = -1, clusterCount;
    private double lastStamp = Double.NaN, lastFresh = Double.NEGATIVE_INFINITY, lastSeen = Double.NEGATIVE_INFINITY, since;
    private State state = State.UNKNOWN, candidate = State.UNKNOWN;
    private int count;
    /** Why the state is what it is, for telemetry and the robot log. */
    public String reason = "No observation";
    /** Last frame's numbers for our cell, for telemetry and the robot log (NaN/0 when not seen). */
    public double lastHeightIn = Double.NaN, lastSidewaysIn = Double.NaN;
    public int lastTagCount;

    /** allowed: the cells that can be ours (allowedCells). Exactly one = fixed from the start. */
    public IsHiveReadyForShot(Mount mount, int... allowed) {
        this.mount = mount; this.allowed = allowed.length == 0 ? ALL_CELLS : allowed.clone();
        if (this.allowed.length == 1) cluster = this.allowed[0];
    }
    /** Our cell's first tag ID once locked (30/34/38/42), else -1. The lock lasts for this object. */
    public int cluster() { return cluster; }
    /** Forgets UP/DOWN (e.g. while driving), keeps which cell is ours. */
    public void reset() {
        state = candidate = State.UNKNOWN; count = 0; lastFresh = lastSeen = Double.NEGATIVE_INFINITY;
        if (cluster < 0) { candidateCluster = -1; clusterCount = 0; }
    }

    public void update(Frame f, double now) {
        if (!Double.isFinite(now)) { reset(); reason = "Invalid clock"; return; }
        if (now - lastFresh > MAX_AGE || now < lastFresh) reset();
        if (f == null || !f.usable()) { reset(); clearLast(); reason = "No usable camera frame"; return; }
        if (f.stamp == lastStamp) { reason = "Waiting for new camera frame"; return; }
        if (Double.isFinite(lastStamp) && f.stamp < lastStamp) reset(); // camera restarted
        lastStamp = f.stamp; lastFresh = now;
        if (cluster < 0 && !identify(f, now)) return;
        clearLast();
        State observed = null;
        double sum = 0;
        String problem = mount.problem();
        for (Tag t : f.tags) {
            // Our cell's tags are straight behind; one far to the side means we're not where we think.
            if (clusterOf(t.id) != cluster || Math.abs(mount.sidewaysIn(t)) > mount.oursMaxSidewaysIn) continue;
            lastTagCount++;
            double side = mount.sidewaysIn(t);
            if (!(Math.abs(side) >= Math.abs(lastSidewaysIn))) lastSidewaysIn = side; // most central tag
            if (!problem.isEmpty()) continue;
            double h = mount.heightIn(t);
            sum += h;
            State s = mount.classify(h);
            observed = observed == null || observed == s ? s : State.UNKNOWN; // tags must agree
        }
        if (lastTagCount > 0 && problem.isEmpty()) lastHeightIn = sum / lastTagCount;
        if (!problem.isEmpty()) { state = State.UNKNOWN; candidate = State.UNKNOWN; count = 0; reason = problem; return; }
        if (lastTagCount == 0) { miss(now); return; }
        lastSeen = now;
        // A frame that is not the current state takes permission away at once.
        if (observed != state) state = State.UNKNOWN;
        if (observed == State.UNKNOWN) {
            candidate = State.UNKNOWN; count = 0;
            reason = String.format(Locale.US, "Between bands: %.1f in", lastHeightIn);
            return;
        }
        if (observed != candidate) { candidate = observed; count = 1; since = now; } else count++;
        if (count >= CONFIRM_FRAMES && now - since >= CONFIRM_SECONDS) state = observed;
        reason = String.format(Locale.US, "%s %s, %.1f in", state == State.UNKNOWN ? "Confirming" : "Stable",
                observed, lastHeightIn);
    }

    /** A new frame without our cell's tags: hold, unless the last sighting is too old. */
    private void miss(double now) {
        if (now - lastSeen > MISS_HOLD) {
            state = candidate = State.UNKNOWN; count = 0;
            reason = "Our cell's tags not in view";
        } else reason = String.format(Locale.US, "Holding %s: our tags missed for %.2f s",
                state != State.UNKNOWN ? state : candidate == State.UNKNOWN ? "UNKNOWN" : "confirming " + candidate,
                now - lastSeen);
    }

    /** Lock our cell after it is the nearest cell in CONFIRM_FRAMES frames that show an allowed cell,
     * with no other cell in between and no gap longer than MISS_HOLD. */
    private boolean identify(Frame f, double now) {
        int c = nearestCluster(f, mount, allowed);
        if (c < 0) {
            if (now - lastSeen > MISS_HOLD) { clusterCount = 0; candidateCluster = -1; }
            clearLast(); reason = "Looking for our hive cell"; return false;
        }
        lastSeen = now;
        if (c != candidateCluster) { candidateCluster = c; clusterCount = 1; } else clusterCount++;
        if (clusterCount < CONFIRM_FRAMES) { clearLast(); reason = "Identifying " + clusterName(c); return false; }
        cluster = c;
        return true;
    }
    private void clearLast() { lastHeightIn = Double.NaN; lastSidewaysIn = Double.NaN; lastTagCount = 0; }

    public State state(double now) {
        return Double.isFinite(now) && now >= lastFresh && now - lastFresh <= MAX_AGE
                && now - lastSeen <= MISS_HOLD ? state : State.UNKNOWN;
    }
    public boolean isHiveReadyForShot(double now) { return state(now) == State.UP; }
}
