package org.firstinspires.ftc.teamcode.pollen;

/** Bridge code, frozen: turns a distance and angle into seconds and wheel powers for timed
 * driving. Odometry and Pedro Pathing will replace it. See docs/POLLEN_MAT_CALIBRATION.md. */
public final class PollenDriveCalibration {
    private PollenDriveCalibration() { }

    public enum Profile { TESTED_BASELINE, MAT_TRIAL }
    public static final double BASELINE_COEFFICIENT = 129;
    // Least squares through the origin using all three 0.25-power forward trials.
    public static final double FORWARD_MAT_COEFFICIENT =
            (.25 * 32.6 + .50 * 68 + .75 * 105) / (.25 * .25 + .50 * .50 + .75 * .75);
    public static final double FORWARD_HALF_POWER_COEFFICIENT = 113 / (.5 * 1.5);
    private static final double OBSERVED_DIAGONAL_DEGREES = Math.toDegrees(Math.atan2(31, 54));
    private static final double DIAGONAL_DISTANCE_RATIO = Math.hypot(54, 31) / 75;

    public static final class Move {
        public final double seconds, forward, strafe, commandAngleDegrees;
        private Move(double seconds, double power, double angle) {
            this.seconds = seconds;
            this.commandAngleDegrees = angle;
            forward = power * Math.cos(Math.toRadians(angle));
            strafe = -power * Math.sin(Math.toRadians(angle));
        }
    }

    /** Interpolate the coefficient between the measured powers; clamp outside them.
     * Outside .25-.50 and between measured durations this is an unverified estimate.
     * A remains time-controlled: this only estimates its displayed distance. */
    public static double forwardDistanceCm(double seconds, double power) {
        if (!Double.isFinite(seconds) || seconds < 0 || !Double.isFinite(power) || power <= 0)
            throw new IllegalArgumentException("Invalid forward estimate");
        double fraction = Math.max(0, Math.min(1, (power - .25) / .25));
        return seconds * power * lerp(FORWARD_MAT_COEFFICIENT,
                FORWARD_HALF_POWER_COEFFICIENT, fraction);
    }

    /** MAT_TRIAL assumes the reported shoot trials used the unchanged 129 model.
     * Their installed settings were NOT confirmed. Do not promote it to baseline.
     * Interpolate measured output angle and distance ratio, then invert for the
     * desired angle. The unmeasured opposite direction is assumed symmetric. */
    public static Move move(Profile profile, double distanceCm, double power, double angleDegrees) {
        if (profile == null || !Double.isFinite(distanceCm) || distanceCm < 0
                || !Double.isFinite(power) || power <= 0 || power > .5
                || !Double.isFinite(angleDegrees) || Math.abs(angleDegrees) > 90)
            throw new IllegalArgumentException("Invalid timed move");
        double commandAngle = angleDegrees, ratio = 1;
        if (profile == Profile.MAT_TRIAL) {
            double desired = Math.abs(angleDegrees), fraction;
            if (desired <= OBSERVED_DIAGONAL_DEGREES) {
                fraction = desired / OBSERVED_DIAGONAL_DEGREES;
                commandAngle = 45 * fraction;
                ratio = lerp(1.07, DIAGONAL_DISTANCE_RATIO, fraction);
            } else {
                fraction = (desired - OBSERVED_DIAGONAL_DEGREES) / (90 - OBSERVED_DIAGONAL_DEGREES);
                commandAngle = lerp(45, 90, fraction);
                ratio = lerp(DIAGONAL_DISTANCE_RATIO, .80, fraction);
            }
            commandAngle = Math.copySign(commandAngle, angleDegrees);
        }
        return new Move(distanceCm / (power * BASELINE_COEFFICIENT * ratio), power, commandAngle);
    }

    private static double lerp(double a, double b, double fraction) { return a + (b - a) * fraction; }
}
