package org.firstinspires.ftc.teamcode.pollen;

import java.util.ArrayList;
import java.util.List;

/** Circle's aim (PollenHiveAim): which cell, which way, and that the turn finishes on a robot that
 * needs a real push to turn. Simulated; not a field check. */
public final class PollenHiveAimTest {
    private static int checks;
    private static void check(boolean ok, String why) { checks++; if (!ok) throw new AssertionError(why); }
    private static final IsHiveReadyForShot.Mount M = PollenHiveTest.M;

    /** A frame of cell 34 (and one blue tag far to the side) seen from a robot whose heading is
     * h, with the cell middle in world direction target (radians CCW), distIn away. */
    private static IsHiveReadyForShot.Frame view(double stamp, double h, double target, double distIn) {
        double right = PollenOdometryDrive.wrap(h - target);
        double side = distIn * Math.sin(right), fwd = distIn * Math.cos(right);
        return new IsHiveReadyForShot.Frame(stamp,
                PollenHiveTest.tag(34, 49.7, side - 6.9, fwd, M), PollenHiveTest.tag(35, 49.7, side - 2.3, fwd, M),
                PollenHiveTest.tag(36, 49.7, side + 2.3, fwd, M), PollenHiveTest.tag(37, 49.7, side + 6.9, fwd, M),
                PollenHiveTest.tag(42, 49.7, side + 30, fwd, M));
    }

    /** Smoothness of the last run(): how often the turn command flipped sign, its biggest jump. */
    private static int flips;
    private static double biggestJump;

    /** Runs one aim on a robot that only turns when |power| >= stiction, with a camera that sees
     * 3 loops (60 ms) late. Returns the final heading. */
    private static double run(PollenHiveAim.Source src, double h, double target, double stiction,
                              double x, double y, boolean odoKnown, List<String> events, double targetLeftIn) {
        PollenHiveAim aim = new PollenHiveAim();
        aim.setTargetLeftIn(targetLeftIn);
        double w = 0, dt = .02, stamp = 1, previous = 0, lastStrong = 0;
        double[] past = {h, h, h};
        flips = 0; biggestJump = 0;
        String e = aim.start(0, src, h, x, y, odoKnown, view(stamp, h, target, 40), 0, M);
        events.add(e);
        for (int i = 1; i <= 400 && aim.active(); i++) {
            double now = i * dt;
            PollenHiveAim.Output o = aim.tick(now, h, w, view(++stamp, past[i % 3], target, 40), 0, M);
            if (o.event != null) events.add(o.event);
            if (aim.active()) {
                // A reversal counts when it is strong enough to move a real robot (about 0.1).
                if (Math.abs(o.turn) >= .08) {
                    if (lastStrong != 0 && Math.signum(lastStrong) != Math.signum(o.turn)) flips++;
                    lastStrong = o.turn;
                }
                if (i > 1) biggestJump = Math.max(biggestJump, Math.abs(o.turn - previous));
                previous = o.turn;
            }
            past[i % 3] = h;
            double cmd = Math.abs(o.turn) >= stiction ? o.turn : 0;
            w += dt / (.08 + dt) * (-cmd * 3.0 - w); // + turn = clockwise = heading down
            h += w * dt;
        }
        check(!aim.active(), "Aim finishes: " + events);
        return h;
    }
    private static double run(PollenHiveAim.Source src, double h, double target, double stiction,
                              double x, double y, boolean odoKnown, List<String> events) {
        return run(src, h, target, stiction, x, y, odoKnown, events, 0);
    }

    public static void main(String[] args) {
        // Which cell and which way.
        IsHiveReadyForShot.Frame f = view(1, 0, Math.toRadians(-7.125), 40);
        PollenHiveAim.Bearing b = PollenHiveAim.bearing(f, M);
        check(b != null && b.cluster == 34 && b.tags == 4 && Math.abs(b.degrees - 7.125) < .05,
                "Cell nearest the launcher line, + = right: " + (b == null ? "none" : b.cluster + " " + b.degrees));
        check(Math.abs(b.distanceIn - 40) < .1, "Horizontal distance from the tilted camera: " + b.distanceIn);
        check(PollenHiveAim.bearing(new IsHiveReadyForShot.Frame(1), M) == null, "No tags, no bearing");
        // v38: the nearest cell, not the one nearest the launcher line (v37 flipped between 34 and 30).
        IsHiveReadyForShot.Frame two = new IsHiveReadyForShot.Frame(1,
                PollenHiveTest.tag(34, 49.7, 4, 45, M), PollenHiveTest.tag(35, 49.7, 8, 45, M),
                PollenHiveTest.tag(30, 49.7, -1, 70, M), PollenHiveTest.tag(31, 49.7, 1, 70, M));
        b = PollenHiveAim.bearing(two, M);
        check(b.cluster == 34, "Aims at the nearest cell, not the far one on the line: " + b.cluster);
        IsHiveReadyForShot.Frame lone = new IsHiveReadyForShot.Frame(1,
                PollenHiveTest.tag(34, 49.7, 4, 45, M), PollenHiveTest.tag(30, 49.7, -1, 70, M), PollenHiveTest.tag(31, 49.7, 1, 70, M));
        check(PollenHiveAim.bearing(lone, M).cluster == 30, "A cell with 2+ tags beats a single tag");
        // Calibration readout: robot at the start with the cell 2 in right of the camera line.
        IsHiveReadyForShot.Frame offCenter = new IsHiveReadyForShot.Frame(1,
                PollenHiveTest.tag(34, 49.7, 2 - 6.9, 50, M), PollenHiveTest.tag(35, 49.7, 2 - 2.3, 50, M),
                PollenHiveTest.tag(36, 49.7, 2 + 2.3, 50, M), PollenHiveTest.tag(37, 49.7, 2 + 6.9, 50, M));
        check(Math.abs(PollenHiveAim.sidewaysToCenter(offCenter, M, 0, 0) - (M.sidewaysCorrectionIn - 2)) < .01,
                "Readout: the correction that centers the cell at the start");
        check(new PollenHiveSettings.Values(PollenHiveSettings.CAMERA, false).get("sideways") == 0,
                "Camera sideways correction is 0 since v38 (camera straightened)");

        // Camera aim from 12 deg off, on a robot that needs 0.12 to turn at all.
        for (double stiction : new double[]{0, .12, .2}) {
            List<String> ev = new ArrayList<>();
            double target = Math.toRadians(-12);
            double h = run(PollenHiveAim.Source.CAMERA, 0, target, stiction, Double.NaN, Double.NaN, false, ev);
            double off = Math.toDegrees(PollenOdometryDrive.wrap(h - target));
            check(Math.abs(off) <= PollenHiveAim.TOLERANCE_DEG + .05, "Camera aim lands on the cell (stiction " + stiction + "): " + off + " " + ev);
            // v36 stuttered: on/off minimum power and stop-and-look-again. One smooth turn now.
            check(flips <= 1 && biggestJump <= .12, "Smooth turn, no stutter (stiction " + stiction + "): flips " + flips + ", jump " + biggestJump);
            String done = ev.get(ev.size() - 1);
            check(ev.get(0).startsWith("AIM_START") && done.startsWith("AIM_DONE") && done.contains("source=CAMERA")
                    && done.contains("first_camera_bearing_deg=12.00") && done.contains("camera_residual_deg="),
                    "Recorded start and result: " + ev);
        }
        // Left of the cell, the turn is to the right (clockwise, heading decreases), and vice versa.
        List<String> ev = new ArrayList<>();
        double h = run(PollenHiveAim.Source.CAMERA, Math.toRadians(20), Math.toRadians(5), .12, Double.NaN, Double.NaN, false, ev);
        check(Math.abs(Math.toDegrees(h) - 5) <= 1.05, "Turns right 15 deg: " + Math.toDegrees(h));

        // Odometry aim from the angled spot (8 in away, 8 in left of the primary start).
        ev.clear();
        h = run(PollenHiveAim.Source.ODOMETRY, 0, 0, .12, 8, 8, true, ev);
        check(Math.abs(Math.toDegrees(h) - PollenFieldGeometry.aimDegrees(8, 8)) <= PollenHiveAim.TOLERANCE_DEG + .1,
                "Odometry aim from 8/8 turns to the computed angle: " + Math.toDegrees(h));
        check(ev.get(0).contains("odo_target_deg=-11.92"), "Odometry target recorded: " + ev.get(0));

        // Aim 2 in right of the middle (the default): the middle ends up 2 in left, about 2.9 deg at 40 in.
        ev.clear();
        h = run(PollenHiveAim.Source.CAMERA, 0, Math.toRadians(-12), .12, Double.NaN, Double.NaN, false, ev, PollenConfig.AIM_TARGET_LEFT_IN);
        double middle = Math.toDegrees(PollenOdometryDrive.wrap(h - Math.toRadians(-12)));
        check(Math.abs(middle + Math.toDegrees(Math.atan2(2, 40))) <= PollenHiveAim.TOLERANCE_DEG + .1,
                "Default aims 2 in right of the cell's middle: " + middle);
        check(ev.get(0).contains("target_left_in=-2.0"), "Offset recorded: " + ev.get(0));
        check(PollenConfig.AIM_TARGET_LEFT_IN == -2 && new PollenHiveAim().targetLeftIn() == -2, "Default target is 2 in right");
        check(Math.abs(PollenFieldGeometry.aimDegrees(8, 8, -2) + Math.toDegrees(Math.atan2(10, PollenFieldGeometry.hiveOpeningForwardInches() - 8))) < 1e-9,
                "Auto aim at 2 in right of the middle from 8/8");

        // Refusals say why, and do nothing.
        PollenHiveAim aim = new PollenHiveAim();
        String r = aim.start(0, PollenHiveAim.Source.CAMERA, 0, 0, 0, false, null, 99, M);
        check(!aim.active() && r.startsWith("AIM_REFUSED") && r.contains("no_tags"), "No camera frame: refuse: " + r);
        r = aim.start(0, PollenHiveAim.Source.ODOMETRY, 0, 3, 4, false, f, 0, M);
        check(!aim.active() && r.contains("odometry_unknown"), "Odometry aim needs the primary start: " + r);
        r = aim.start(0, PollenHiveAim.Source.CAMERA, 0, 0, 0, false, f, PollenHiveAim.MAX_FRAME_AGE_S + .1, M);
        check(!aim.active(), "An old frame cannot start a camera aim");
        aim.start(0, PollenHiveAim.Source.CAMERA, 0, 0, 0, false, f, 0, M);
        String c = aim.cancel(.1, 0, "driver_turned");
        check(!aim.active() && c.startsWith("AIM_CANCEL") && c.contains("driver_turned"), "Driver can cancel: " + c);

        // A robot that cannot turn gives up on time and says so.
        aim = new PollenHiveAim();
        aim.start(0, PollenHiveAim.Source.CAMERA, 0, 0, 0, false, f, 0, M);
        String last = null;
        for (int i = 1; i <= 200 && aim.active(); i++) {
            PollenHiveAim.Output o = aim.tick(i * .02, 0, 0, f, 0, M);
            if (o.event != null) last = o.event;
            if (i > 50) check(Math.abs(o.turn) >= PollenHiveAim.KS || !aim.active(), "Stuck: the push keeps growing: " + o.turn);
        }
        check(!aim.active() && last != null && last.contains("turn_timeout"), "Stuck robot times out: " + last);
        System.out.println("PASS PollenHiveAimTest: " + checks + " checks (simulated turns, not a field check)");
    }
}
