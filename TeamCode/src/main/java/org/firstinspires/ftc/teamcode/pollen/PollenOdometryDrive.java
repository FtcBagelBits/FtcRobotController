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
 * slowing down evenly as it gets close (power proportional to the square root of the distance
 * left, which is a steady braking rate), while holding the heading it was zeroed at (nothing
 * turns the chassis).
 * Pedro Pathing's path follower can replace it once Pedro is tuned for this robot (AutoTune); the
 * pose already comes from Pedro's Pinpoint localizer.
 */
public final class PollenOdometryDrive {
    private PollenOdometryDrive() { }

    public static final double CM_PER_INCH = 2.54;

    /** A leg is done once the robot is this close to its end point (inches) AND has stopped
     * (moving slower than SETTLED_IN_PER_SECOND). Inside the tolerance the power is zero; if the
     * robot coasts back out, the controller drives it back. */
    public static final double POSITION_TOLERANCE_IN = 0.5, SETTLED_IN_PER_SECOND = 1.5;
    /** Slow-down: power = 0.20 x sqrt(inches still to go), so 0.6 power until 9 in out, 0.4 at
     * 4 in, 0.2 at 1 in. Bigger = brakes later and harder. (Slowing in proportion to the distance
     * instead creeps for 1.5-2 s at the end of every leg.) */
    public static final double SLOWDOWN_POWER_PER_SQRT_INCH = 0.20;
    /** The robot keeps rolling for a moment after the power drops, so braking is based on where it
     * will be this many seconds from now at its current speed, not where it is. Bigger = brakes
     * earlier. */
    public static final double LOOKAHEAD_SECONDS = 0.15;
    /** Least power used while outside the tolerance, so the robot does not stall just short. */
    public static final double MIN_POWER = 0.08;
    /** Default leg power for the odometry autos, and the most the INIT menus allow. Faster than
     * the timed autos (0.20-0.45): odometry measures where the robot is, so the slow-down can
     * correct for speed. Faster still costs something: wheel slip, overshoot and pod bounce grow
     * with speed, so check stopping on field tiles before raising it. */
    public static final double DEFAULT_LEG_POWER = 0.60, MAX_LEG_POWER = 0.80;
    /** Power may rise this fast at the start of a leg (per second), so the wheels don't spin. */
    public static final double RAMP_POWER_PER_SECOND = 2.0;
    /** Turn power per radian of heading error, to keep the zeroed heading; capped at MAX_TURN. */
    public static final double HEADING_KP = 1.0, MAX_TURN = 0.25;

    /** For TIME ESTIMATES only (the budget and each leg's give-up time), never for driving.
     * If the tuned timed routes reached their planned park spots, they moved at about 125-150
     * cm/s per unit of power; this is set a little slower than both. */
    public static final double ESTIMATE_CM_PER_SECOND_PER_POWER = 120;
    /** Added to each leg's estimate for slowing down at the end. */
    public static final double LEG_SETTLE_SECONDS = 0.2;
    /** A leg that has not arrived after (2 x estimate + 1 s) has run out of time. If it is within
     * NEAR_ENOUGH_IN of its end point the next leg starts; otherwise the whole route STOPS. Never
     * just start the next leg from far away: the legs are planned to go around things (wall, hive,
     * flower), and a straight line from where a stuck robot is could cut right through them. */
    public static final double TIMEOUT_FACTOR = 2, TIMEOUT_EXTRA_SECONDS = 1;
    /** "Almost there": a leg this close to its end point may hand over to the next leg. */
    public static final double NEAR_ENOUGH_IN = 2.0;
    /** Stuck check: if the wheels are being driven, the robot is farther than NEAR_ENOUGH_IN from
     * the end point, and odometry shows it moved less than NO_PROGRESS_MIN_IN in
     * NO_PROGRESS_SECONDS, something is wrong (blocked robot, unplugged pod, frozen reading):
     * stop everything. */
    public static final double NO_PROGRESS_SECONDS = 0.75, NO_PROGRESS_MIN_IN = 0.5;

    /** Wall squaring (B and C, before the first leg): back gently into the end wall so the
     * robot's back sits flat against it, then zero the odometry there. The wall stops the robot;
     * this is a short push by time, not a distance, because a distance the wall prevents
     * would never be reached. */
    public static final double SQUARE_POWER = 0.30, SQUARE_SECONDS = 0.35;
    /** Zero-output pause right after zeroing, so no loop acts on a pose read from before the zero. */
    public static final double ZERO_SETTLE_SECONDS = 0.10;

    /** Pinpoint statuses that mean the hardware is not working (a pod not plugged in, or the IMU
     * spinning by itself). INIT shows PINPOINT FAULT so the drive team can switch to a timed mode.
     * START still runs, so the shot still happens; the stuck check then stops the driving.
     * FAULT_BAD_READ is not here: a single bad read happens and the next one is fine (the stuck
     * check catches a reading that stays frozen). Takes the status's name so this is testable. */
    public static boolean pinpointFault(String status) {
        return status != null && status.startsWith("FAULT_") && !status.equals("FAULT_BAD_READ");
    }

    /** Seconds a leg is expected to take: used for the time budget. */
    public static double plannedSeconds(double distanceCm, double maxPower) {
        check(distanceCm, maxPower);
        return distanceCm / (maxPower * ESTIMATE_CM_PER_SECOND_PER_POWER) + LEG_SETTLE_SECONDS;
    }

    /** Seconds after which a leg gives up. */
    public static double timeoutSeconds(double distanceCm, double maxPower) {
        return TIMEOUT_FACTOR * plannedSeconds(distanceCm, maxPower) + TIMEOUT_EXTRA_SECONDS;
    }

    /** What {@link #command} returns: this loop's drive powers, and how far the leg has left to go. */
    public static final class Command {
        /** Drive powers in PollenSequence's directions: forward +, strafe + = RIGHT, turn + = CLOCKWISE. */
        public final double forward, strafe, turn;
        /** Straight-line inches from the robot to the leg's end point. */
        public final double inchesToGo;

        Command(double forward, double strafe, double turn, double inchesToGo) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            this.inchesToGo = inchesToGo;
        }

        /** False when all three powers are zero: inside the tolerance, or coasting up to the end point. */
        public boolean driving() { return forward != 0 || strafe != 0 || turn != 0; }
    }

    /** Same as the command below, for a robot that is standing still. */
    public static Command command(double targetForwardIn, double targetLeftIn, double maxPower,
                                  double x, double y, double heading) {
        return command(targetForwardIn, targetLeftIn, maxPower, x, y, heading, 0, 0);
    }

    /**
     * Drive command toward (targetForwardIn, targetLeftIn) from pose (x, y, heading), with the
     * robot moving at (vx, vy) inches per second in the same frame.
     * All three powers are 0 once within tolerance, or when coasting LOOKAHEAD_SECONDS would
     * carry the robot to the end point.
     */
    public static Command command(double targetForwardIn, double targetLeftIn, double maxPower,
                                  double x, double y, double heading, double vx, double vy) {
        double toGoForward = targetForwardIn - x, toGoLeft = targetLeftIn - y;
        double distance = Math.hypot(toGoForward, toGoLeft);
        if (distance <= POSITION_TOLERANCE_IN) return new Command(0, 0, 0, distance);
        // How far it will still be after coasting LOOKAHEAD_SECONDS toward the end point.
        double towardSpeed = (vx * toGoForward + vy * toGoLeft) / distance;
        double brakingDistance = distance - Math.max(0, towardSpeed) * LOOKAHEAD_SECONDS;
        // Cut power only if it would coast right up to the end point anyway; cutting it any
        // earlier leaves the robot stopping short and creeping in.
        if (brakingDistance <= 0) return new Command(0, 0, 0, distance);

        // The error is measured on the field (zeroed frame); the wheels need it relative to the
        // robot, so turn it by the robot's heading.
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double robotForward = cos * toGoForward + sin * toGoLeft;
        double robotLeft = -sin * toGoForward + cos * toGoLeft;

        double power = Math.min(maxPower, Math.max(Math.min(MIN_POWER, maxPower),
                SLOWDOWN_POWER_PER_SQRT_INCH * Math.sqrt(brakingDistance)));
        double forward = power * robotForward / distance;
        double strafe = -power * robotLeft / distance; // strafe + is RIGHT
        // Heading counterclockwise of zero -> turn clockwise (turn +) to come back.
        double turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, HEADING_KP * wrap(heading)));
        return new Command(forward, strafe, turn, distance);
    }

    /** Leg power allowed {@code secondsIntoLeg} after the leg started: ramps up from MIN_POWER. */
    public static double rampedPower(double secondsIntoLeg, double maxPower) {
        return Math.min(maxPower, MIN_POWER + RAMP_POWER_PER_SECOND * Math.max(0, secondsIntoLeg));
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
