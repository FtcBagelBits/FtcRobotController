package org.firstinspires.ftc.teamcode.pollen;

/** Robot-relative PollenBot stick mix, with a hold-to-drive gate for camera exploration. */
public final class PollenCameraDriveControl {
    public static final double NORMAL_POWER = .30, FINE_POWER = .15;
    private boolean released;

    /** Call at START: holding L1 across INIT/START must not move the robot. */
    public void reset() { released = false; }

    /** Order matches PollenConfig.DRIVE_NAMES: FL, FR, BL, BR. */
    public double[] update(boolean enable, boolean fine, double forward, double strafe, double turn) {
        if (!enable) { released = true; return new double[4]; }
        if (!released || !Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(turn))
            return new double[4];
        forward = stick(forward); strafe = stick(strafe); turn = stick(turn);
        double[] powers = {forward + strafe + turn, forward - strafe - turn,
                forward - strafe + turn, forward + strafe - turn};
        double max = 1;
        for (double power : powers) max = Math.max(max, Math.abs(power));
        double scale = (fine ? FINE_POWER : NORMAL_POWER) / max;
        for (int i = 0; i < powers.length; i++) powers[i] *= scale;
        return powers;
    }

    private static double stick(double value) {
        return Math.abs(value) < .05 ? 0 : Math.max(-1, Math.min(1, value));
    }
}
