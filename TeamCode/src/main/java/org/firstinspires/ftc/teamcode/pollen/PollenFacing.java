package org.firstinspires.ftc.teamcode.pollen;

/** Which way the robot faces at the start of autonomous, and how a route becomes the real
 * robot's directions. Pure Java, no hardware, so it is tested on a computer.
 *
 * Every route is WRITTEN as if the robot faced the hive: forward = toward the hive, left = left
 * as seen from the start spot looking at the hive, "square to the wall" = push toward the end
 * wall. These "route directions" are what the INIT menus, leg angles, leg end points, the time
 * budget, the field-route checker and the robot log use.
 *
 * Since pollen-odo-v11 the launcher (topspin) shoots out of the robot's BACK, so the robot starts
 * the other way round: FRONT flat against the end wall, BACK to the hive. Then the robot's own
 * forward is the route's backward, and its own left is the route's right. Clockwise is still
 * clockwise (turning the robot around does not change which way it spins), so turns and headings
 * stay the same. The robot drives the very same path on the field; it just drives it backward.
 *
 * Every autonomous OpMode uses this in exactly two places:
 *   1. READ: the odometry pose goes through routePose() before the sequence sees it.
 *   2. ACT:  the sequence's output goes through wheelPowers() on its way to the drive motors.
 * Nothing else knows which way the robot faces. To go back to a launcher on the front, set
 * START_BACK_TO_HIVE to false (and change the placement in the drive-team guides).
 */
public final class PollenFacing {
    private PollenFacing() { }

    /** true: the robot starts with its front against the end wall and its back (the launcher) to
     * the hive. false: the old robot, back against the wall and front (launcher) to the hive. */
    public static final boolean START_BACK_TO_HIVE = true;

    /** Odometry pose (inches, radians, from the last zero, in the robot's OWN directions: x
     * forward, y left, heading counterclockwise) to route directions {x, y, heading}. */
    public static double[] routePose(double x, double y, double heading) {
        return routePose(x, y, heading, START_BACK_TO_HIVE);
    }

    public static double[] routePose(double x, double y, double heading, boolean backToHive) {
        return backToHive ? new double[] {flip(x), flip(y), heading} : new double[] {x, y, heading};
    }

    /** The four drive motor powers (frontLeft, frontRight, backLeft, backRight) for this output,
     * whose forward and strafe are in route directions. */
    public static double[] wheelPowers(PollenSequence.Output out) {
        return wheelPowers(out, START_BACK_TO_HIVE);
    }

    public static double[] wheelPowers(PollenSequence.Output out, boolean backToHive) {
        if (!backToHive) return PollenSequence.wheelPowers(out);
        return PollenSequence.wheelPowers(flip(out.forward), flip(out.strafe), out.turn);
    }

    /** For the INIT screens and the robot log. */
    public static String placement() {
        return START_BACK_TO_HIVE ? "FRONT flat against the end wall, BACK (launcher) to the hive"
                : "BACK flat against the end wall, FRONT (launcher) to the hive";
    }

    /** What wall squaring does, in the robot's own directions. */
    public static String squareMove() {
        return START_BACK_TO_HIVE ? "drive FORWARD into the end wall" : "back into the end wall";
    }

    /** Which way the robot itself moves when a route goes "forward" (toward the hive). */
    public static String routeForward() {
        return START_BACK_TO_HIVE ? "BACKWARD (launcher end first)" : "forward";
    }

    /** For the robot log's START line. */
    public static String logName() {
        return START_BACK_TO_HIVE ? "back-to-hive" : "front-to-hive";
    }

    /** -v, but never -0.0 (which would print as "-0.00" on the screen and in the log). */
    private static double flip(double v) {
        return v == 0 ? 0 : -v;
    }
}
