package org.firstinspires.ftc.teamcode.pollen;

/**
 * Odometry driving for the Pollen autos: given where the robot is and where the leg ends,
 * return the drive powers for this loop. Pure Java, no hardware, so it can be tested off the robot.
 *
 * Pose: inches and radians, measured from where the odometry was last zeroed, in the
 * Pinpoint/Pedro directions: x forward, y LEFT, heading counterclockwise-positive.
 * Outputs: PollenSequence's directions: forward +, strafe + = RIGHT, turn + = CLOCKWISE.
 *
 * The controller is deliberately simple: drive straight at the end point, fast when far away and
 * slower as it gets close, while holding the heading it was zeroed at (nothing turns the chassis).
 * Pedro Pathing's path follower can replace it once Pedro is tuned for this robot (AutoTune); the
 * pose already comes from Pedro's Pinpoint localizer.
 */
public final class PollenOdometryDrive {
    private PollenOdometryDrive() { }

    public static final double CM_PER_INCH = 2.54;

    /** A leg is done once the robot is this close to its end point (inches). */
    public static final double POSITION_TOLERANCE_IN = 0.5;
    /** Power per inch still to go: full leg power until about (power / 0.04) inches out. */
    public static final double KP_POWER_PER_INCH = 0.04;
    /** Least power used while outside the tolerance, so the robot does not stall just short. */
    public static final double MIN_POWER = 0.08;
    /** Turn power per radian of heading error, to keep the zeroed heading; capped at MAX_TURN. */
    public static final double HEADING_KP = 1.0, MAX_TURN = 0.25;

    /** For TIME ESTIMATES only (the budget and each leg's give-up time), never for driving.
     * If the tuned timed routes reached their planned park spots, they moved at about 125-150
     * cm/s per unit of power; this is set a little slower than both. */
    public static final double ESTIMATE_CM_PER_SECOND_PER_POWER = 120;
    /** Added to each leg's estimate for slowing down at the end. */
    public static final double LEG_SETTLE_SECONDS = 0.2;
    /** A leg that has not arrived after (2 x estimate + 1 s) gives up and the next leg starts.
     * Legs end at fixed points, so the next leg still heads for its own correct end point. */
    public static final double TIMEOUT_FACTOR = 2, TIMEOUT_EXTRA_SECONDS = 1;

    /** Wall squaring (B and C, before the first leg): back gently into the end wall so the
     * robot's back sits flat against it, then zero the odometry there. The wall stops the robot;
     * this is a short push by time, not a distance, because a distance the wall prevents
     * would never be reached. */
    public static final double SQUARE_POWER = 0.30, SQUARE_SECONDS = 0.35;
    /** Zero-output pause right after zeroing, so no loop acts on a pose read from before the zero. */
    public static final double ZERO_SETTLE_SECONDS = 0.10;

    /** Seconds a leg is expected to take: used for the time budget. */
    public static double plannedSeconds(double distanceCm, double maxPower) {
        check(distanceCm, maxPower);
        return distanceCm / (maxPower * ESTIMATE_CM_PER_SECOND_PER_POWER) + LEG_SETTLE_SECONDS;
    }

    /** Seconds after which a leg gives up. */
    public static double timeoutSeconds(double distanceCm, double maxPower) {
        return TIMEOUT_FACTOR * plannedSeconds(distanceCm, maxPower) + TIMEOUT_EXTRA_SECONDS;
    }

    /**
     * Drive command toward (targetForwardIn, targetLeftIn) from pose (x, y, heading).
     * @return {forward, strafe, turn, inchesToGo}; all three powers are 0 once within tolerance.
     */
    public static double[] command(double targetForwardIn, double targetLeftIn, double maxPower,
                                   double x, double y, double heading) {
        double toGoForward = targetForwardIn - x, toGoLeft = targetLeftIn - y;
        double distance = Math.hypot(toGoForward, toGoLeft);
        if (distance <= POSITION_TOLERANCE_IN) return new double[] {0, 0, 0, distance};

        // The error is measured on the field (zeroed frame); the wheels need it relative to the
        // robot, so turn it by the robot's heading.
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double robotForward = cos * toGoForward + sin * toGoLeft;
        double robotLeft = -sin * toGoForward + cos * toGoLeft;

        double power = Math.min(maxPower, Math.max(Math.min(MIN_POWER, maxPower), distance * KP_POWER_PER_INCH));
        double forward = power * robotForward / distance;
        double strafe = -power * robotLeft / distance; // strafe + is RIGHT
        // Heading counterclockwise of zero -> turn clockwise (turn +) to come back.
        double turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, HEADING_KP * wrap(heading)));
        return new double[] {forward, strafe, turn, distance};
    }

    /** End point of a leg that starts at (fromForwardIn, fromLeftIn) and goes distanceCm at
     * angleDegrees (+ left of the zeroed heading, 0 = straight ahead, - right). */
    public static double[] legEnd(double fromForwardIn, double fromLeftIn, double distanceCm, double angleDegrees) {
        double inches = distanceCm / CM_PER_INCH, a = Math.toRadians(angleDegrees);
        return new double[] {fromForwardIn + inches * Math.cos(a), fromLeftIn + inches * Math.sin(a)};
    }

    public static double wrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private static void check(double distanceCm, double maxPower) {
        if (!Double.isFinite(distanceCm) || distanceCm < 0 || !Double.isFinite(maxPower)
                || maxPower <= 0 || maxPower > 1) throw new IllegalArgumentException("Invalid odometry leg");
    }
}
