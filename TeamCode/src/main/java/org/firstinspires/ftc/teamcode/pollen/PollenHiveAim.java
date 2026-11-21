package org.firstinspires.ftc.teamcode.pollen;

import java.util.Locale;

/**
 * Circle in TeleOp: turn the robot in place so the launcher points at the middle of a hive cell.
 *
 * WHERE TO POINT, two ways, both aiming PollenConfig.AIM_TARGET_LEFT_IN (an INIT row) left of
 * the cell's middle (default -2: 2 in right, as the robot shoots a little left):
 *  - CAMERA (PollenBot and the calibration mode): the Limelight sees a hive cell's AprilTags. The
 *    cell whose middle is nearest the launcher's line is the target. Its bearing (degrees, + = to
 *    the right) sets a target heading; the turn runs on the Pinpoint heading, which updates every
 *    loop with no camera delay, and each new camera frame while turning slowly nudges the target.
 *  - ODOMETRY (calibration mode only): the robot was INIT'ed at the primary shooter start, so the
 *    Pinpoint knows where it is; the aim is PollenFieldGeometry.aimDegrees toward our cell's opening.
 *    In a match TeleOp the Pinpoint starts wherever INIT happened, so this is not used there.
 *
 * v37: one smooth proportional turn like last season's camera aim (no minimum-power steps and no
 * stop-and-look-again), because the v36 turn stuttered.
 *
 * Every turn is reported as text for the TeleOp recording (kind AIM), so turns can be compared and
 * fed back. Pure Java; the OpMode reads the hardware and passes the numbers in.
 *
 * Directions: heading is radians counterclockwise (route frame, PollenFacing); turn power + =
 * clockwise, like every other turn here. A tag to the camera's right needs a clockwise turn,
 * wherever the camera is mounted, because a clockwise turn moves every direction to its right.
 */
public final class PollenHiveAim {
    public enum Source { CAMERA, ODOMETRY }

    /** Done when within TOLERANCE_DEG and turning slower than SETTLED_DEG_S for SETTLE_S. */
    public static final double TOLERANCE_DEG = .4, SETTLED_DEG_S = 8, SETTLE_S = .15;
    /** Smooth turn, as in last season's camera aim (lox/MecanumDriveTrain: clip(bearing x 0.04,
     * +-0.5)): power = KP x degrees off - KD x turn rate + a small push KS toward the target,
     * clipped to MAX_POWER. No on/off minimum and no stop-and-start, so it does not stutter.
     * While stuck (slower than STALL_DEG_S, still outside the tolerance) the push grows smoothly
     * by KS_RAMP_PER_S up to KS_MAX: too weak to move = push a little harder. */
    public static final double KP = .04, KD = .0035, KS = .06, KS_RAMP_PER_S = .1, KS_MAX = .25,
            MAX_POWER = .5, STALL_DEG_S = 3, TIMEOUT_S = 3;
    /** Camera aim: each new frame while turning slower than UPDATE_MAX_DEG_S moves the target
     * UPDATE_GAIN of the way to what the camera now says (smooth, no stop to look again). */
    public static final double UPDATE_MAX_DEG_S = 30, UPDATE_GAIN = .5;
    /** Camera frames older than this cannot start a camera aim. */
    public static final double MAX_FRAME_AGE_S = .3;
    /** Kept for the recording header: the camera re-aims while turning, no separate recheck. */
    public static final double RECHECK_DEG = TOLERANCE_DEG;

    /** Where a hive cell is, seen by the camera. */
    public static final class Bearing {
        public final int cluster, tags;
        /** + = cell middle to the right of the launcher line, degrees. */
        public final double degrees, distanceIn;
        Bearing(int cluster, int tags, double degrees, double distanceIn) {
            this.cluster = cluster; this.tags = tags; this.degrees = degrees; this.distanceIn = distanceIn;
        }
    }

    /** The NEAREST hive cell in this frame (v38; v37 took the one nearest the launcher's line,
     * which flipped between our cell and the far one), preferring cells with 2+ tags, or null.
     * Uses the mean of the cell's visible tags; the mount's sideways correction moves the
     * camera's line onto the launcher's. */
    public static Bearing bearing(IsHiveReadyForShot.Frame f, IsHiveReadyForShot.Mount m) {
        return bearing(f, m, 0);
    }
    /** Bearing to a point targetLeftIn left of the cell's middle (negative = right of it). */
    public static Bearing bearing(IsHiveReadyForShot.Frame f, IsHiveReadyForShot.Mount m, double targetLeftIn) {
        return bearing(f, m, targetLeftIn, IsHiveReadyForShot.ALL_CELLS);
    }
    /** Same, but only these cells (first tag IDs) can be the target, e.g. PollenAlliance.cells. */
    public static Bearing bearing(IsHiveReadyForShot.Frame f, IsHiveReadyForShot.Mount m, double targetLeftIn, int[] cells) {
        if (f == null || m == null) return null;
        double tilt = Double.isFinite(m.tiltUpDeg) ? Math.toRadians(m.tiltUpDeg) : 0;
        Bearing best = null;
        for (int cluster : cells) {
            double side = 0, forward = 0; int n = 0;
            for (IsHiveReadyForShot.Tag t : f.tags) {
                if (IsHiveReadyForShot.clusterOf(t.id) != cluster) continue;
                side += m.sidewaysIn(t);
                // Horizontal distance out of the lens (camera tilted up by tilt; y is down).
                forward += (t.z * Math.cos(tilt) + t.y * Math.sin(tilt)) * IsHiveReadyForShot.METRES_TO_INCHES;
                n++;
            }
            if (n == 0 || forward <= 0) continue;
            side /= n; forward /= n;
            side -= targetLeftIn; // aim this far left of the middle
            Bearing b = new Bearing(cluster, n, Math.toDegrees(Math.atan2(side, forward)), Math.hypot(side, forward));
            boolean better;
            if (best == null) better = true;
            else if ((b.tags >= 2) != (best.tags >= 2)) better = b.tags >= 2; // 2+ tags beat 1 tag
            else better = b.distanceIn < best.distanceIn;                   // then the nearest
            if (better) best = b;
        }
        return best;
    }

    /** Calibration: the camera sideways correction (inches) that would put this cell's middle
     * straight ahead of a robot standing at route (x, y, h) after INIT at the primary start, where
     * the cell's middle is on the start line (route y = 0). Current correction + what is missing. */
    public static double sidewaysToCenter(IsHiveReadyForShot.Frame f, IsHiveReadyForShot.Mount m,
                                          double yIn, double h) {
        Bearing b = bearing(f, m, 0);
        if (b == null || !Double.isFinite(yIn) || !Double.isFinite(h)) return Double.NaN;
        double forward = b.distanceIn * Math.cos(Math.toRadians(b.degrees));
        double side = b.distanceIn * Math.sin(Math.toRadians(b.degrees)); // right, after the correction
        double expectedRight = yIn * Math.cos(h) + forward * Math.sin(h);
        return m.sidewaysCorrectionIn + expectedRight - side;
    }

    /** Odometry aim from the primary start: heading (degrees, +left) toward our cell's opening. */
    public static double odometryAimDegrees(double xIn, double yIn) {
        return PollenFieldGeometry.aimDegrees(xIn, yIn);
    }
    public static double odometryAimDegrees(double xIn, double yIn, double targetLeftIn) {
        return PollenFieldGeometry.aimDegrees(xIn, yIn, targetLeftIn);
    }
    /** Inches left of the cell's middle to aim at (PollenConfig.AIM_TARGET_LEFT_IN by default). */
    private double targetLeftIn = PollenConfig.AIM_TARGET_LEFT_IN;
    public void setTargetLeftIn(double inches) { targetLeftIn = inches; }
    /** Cells Circle may aim at: the alliance saved by the last auto (PollenAlliance), else all. */
    private int[] cells = IsHiveReadyForShot.ALL_CELLS.clone();
    public void setCells(int[] cells) { this.cells = cells == null || cells.length == 0 ? IsHiveReadyForShot.ALL_CELLS.clone() : cells.clone(); }
    public double targetLeftIn() { return targetLeftIn; }

    public static final class Output {
        public double turn;
        public boolean active;
        /** Text for one AIM row in the recording when something happened this loop, else null. */
        public String event;
        public String screen = "";
    }

    private boolean running;
    private Source source;
    private double target, startHeading, began, push, settledSince = Double.NaN, firstBearing, odoTarget, lastStamp = Double.NaN;
    private int updates, count;
    private String last = "Circle = aim at the hive";

    public boolean active() { return running; }
    public String lastResult() { return last; }
    public int count() { return count; }

    /**
     * Circle pressed. h = route heading (radians); x/y = route position (inches) or NaN;
     * odometryKnown = the Pinpoint was zeroed at the primary start. Returns the event text
     * (also when the aim cannot start, which says why).
     */
    public String start(double now, Source src, double h, double x, double y, boolean odometryKnown,
                        IsHiveReadyForShot.Frame frame, double frameAgeS, IsHiveReadyForShot.Mount mount) {
        Bearing b = frameAgeS <= MAX_FRAME_AGE_S ? bearing(frame, mount, targetLeftIn, cells) : null;
        boolean odoOk = odometryKnown && Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(h);
        odoTarget = odoOk ? Math.toRadians(odometryAimDegrees(x, y, targetLeftIn)) : Double.NaN;
        String both = String.format(Locale.US, "target_left_in=%.1f camera_bearing_deg=%s cell=%s tags=%d cam_dist_in=%s odo_target_deg=%s odo_turn_deg=%s x_in=%s y_in=%s heading_deg=%.2f",
                targetLeftIn, b == null ? "none" : fmt(b.degrees), b == null ? "none" : String.valueOf(b.cluster), b == null ? 0 : b.tags,
                b == null ? "none" : fmt(b.distanceIn), odoOk ? fmt(Math.toDegrees(odoTarget)) : "unknown",
                odoOk ? fmt(-Math.toDegrees(PollenOdometryDrive.wrap(odoTarget - h))) : "unknown",
                fmt(x), fmt(y), Math.toDegrees(h));
        if (!Double.isFinite(h)) { last = "NO AIM: no Pinpoint heading"; return "AIM_REFUSED source=" + src + " reason=no_heading " + both; }
        if (src == Source.CAMERA && b == null) {
            last = cells.length == IsHiveReadyForShot.ALL_CELLS.length ? "NO AIM: no hive tags in a fresh camera frame"
                    : "NO AIM: no tags of our alliance's hive cells in a fresh camera frame";
            return "AIM_REFUSED source=CAMERA reason=no_tags " + both;
        }
        if (src == Source.ODOMETRY && !odoOk) {
            last = "NO AIM: odometry start unknown";
            return "AIM_REFUSED source=ODOMETRY reason=odometry_unknown " + both;
        }
        source = src; count++; updates = 0; began = now; startHeading = h; running = true;
        push = KS; settledSince = Double.NaN; lastStamp = frame == null ? Double.NaN : frame.stamp;
        firstBearing = b == null ? Double.NaN : b.degrees;
        target = src == Source.CAMERA ? h - Math.toRadians(b.degrees) : odoTarget;
        return String.format(Locale.US, "AIM_START n=%d source=%s target_deg=%.2f ", count, src, Math.toDegrees(target)) + both;
    }

    /** Driver moved the turn stick, pressed Circle again, or STOP: give the robot back. */
    public String cancel(double now, double h, String why) {
        if (!running) return null;
        running = false;
        last = "AIM CANCELLED: " + why;
        return String.format(Locale.US, "AIM_CANCEL n=%d reason=%s turned_deg=%.2f after_s=%.2f", count, why,
                -Math.toDegrees(PollenOdometryDrive.wrap(h - startHeading)), now - began);
    }

    /** Every loop while active. rate = heading change, radians per second (counterclockwise +). */
    public Output tick(double now, double h, double rate, IsHiveReadyForShot.Frame frame, double frameAgeS,
                       IsHiveReadyForShot.Mount mount) {
        Output o = new Output();
        o.active = running;
        if (!running) { o.screen = last; return o; }
        double rateDeg = Double.isFinite(rate) ? Math.toDegrees(rate) : 0;
        // Camera: a NEW frame while turning slowly nudges the target toward what it now sees.
        if (source == Source.CAMERA && frame != null && frameAgeS <= MAX_FRAME_AGE_S && frame.stamp != lastStamp
                && Math.abs(rateDeg) <= UPDATE_MAX_DEG_S) {
            lastStamp = frame.stamp;
            Bearing b = bearing(frame, mount, targetLeftIn, cells);
            if (b != null) {
                double seen = h - Math.toRadians(b.degrees);
                target += UPDATE_GAIN * PollenOdometryDrive.wrap(seen - target);
                updates++;
            }
        }
        double offDeg = Math.toDegrees(PollenOdometryDrive.wrap(h - target)); // + = counterclockwise of target
        double deg = Math.abs(offDeg);
        if (deg <= TOLERANCE_DEG && Math.abs(rateDeg) <= SETTLED_DEG_S) {
            if (Double.isNaN(settledSince)) settledSince = now;
            if (now - settledSince >= SETTLE_S) return finish(o, now, h, frame, frameAgeS, mount, "done");
        } else settledSince = Double.NaN;
        if (now - began > TIMEOUT_S) return finish(o, now, h, frame, frameAgeS, mount, "turn_timeout");
        // Stuck outside the tolerance: grow the push smoothly (no steps, no on/off).
        if (deg > TOLERANCE_DEG && Math.abs(rateDeg) < STALL_DEG_S) push = Math.min(KS_MAX, push + KS_RAMP_PER_S * .02);
        // Counterclockwise of the target -> turn clockwise (+). Rate counterclockwise + ; damping opposes it.
        // The push fades in from half the tolerance to 1.5x it, so it never switches on and off.
        double fade = Math.max(0, Math.min(1, (deg - .5 * TOLERANCE_DEG) / TOLERANCE_DEG));
        double p = KP * offDeg + KD * rateDeg + Math.signum(offDeg) * push * fade;
        o.turn = Math.max(-MAX_POWER, Math.min(MAX_POWER, p));
        o.screen = String.format(Locale.US, "AIMING (%s): %.1f deg to go, power %.2f", source, offDeg, o.turn);
        return o;
    }

    private Output finish(Output o, double now, double h, IsHiveReadyForShot.Frame frame, double frameAgeS,
                          IsHiveReadyForShot.Mount mount, String why) {
        running = false;
        o.turn = 0; o.active = false;
        Bearing after = frameAgeS <= MAX_FRAME_AGE_S ? bearing(frame, mount, targetLeftIn, cells) : null;
        double turned = -Math.toDegrees(PollenOdometryDrive.wrap(h - startHeading));
        double odoAfter = Double.isFinite(odoTarget) ? Math.toDegrees(PollenOdometryDrive.wrap(h - odoTarget)) : Double.NaN;
        last = String.format(Locale.US, "AIM %d %s: turned %.1f deg right in %.2f s; camera now %s deg off",
                count, why, turned, now - began, after == null ? "?" : fmt(after.degrees));
        o.screen = last;
        o.event = String.format(Locale.US, "AIM_DONE n=%d source=%s result=%s turned_right_deg=%.2f seconds=%.2f camera_updates=%d target_left_in=%.1f first_camera_bearing_deg=%s camera_residual_deg=%s odo_target_deg=%s odo_residual_deg=%s heading_deg=%.2f",
                count, source, why, turned, now - began, updates, targetLeftIn, fmt(firstBearing), after == null ? "none" : fmt(after.degrees),
                Double.isFinite(odoTarget) ? fmt(Math.toDegrees(odoTarget)) : "unknown", fmt(odoAfter), Math.toDegrees(h));
        return o;
    }

    private static String fmt(double v) { return Double.isFinite(v) ? String.format(Locale.US, "%.2f", v) : "NaN"; }
}
