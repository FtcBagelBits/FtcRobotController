package org.firstinspires.ftc.teamcode.pollen;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Opens the Pinpoint through Pedro Pathing's localizer, with the settings in
 * pedroPathing/Constants (the same ones the Odometry Check OpMode verifies). */
public final class PollenOdometryHardware {
    private PollenOdometryHardware() { }

    /** Call during INIT with the robot still: the Pinpoint recalibrates its IMU here. */
    public static PinpointLocalizer open(HardwareMap hardwareMap) {
        String name = Constants.localizerConfig.name.get();
        if (hardwareMap.tryGet(GoBildaPinpointDriver.class, name) == null) {
            throw new IllegalStateException("No goBILDA Pinpoint named \"" + name
                    + "\" in the active configuration; odometry autos cannot run (use the EMERGENCY Timed autos)");
        }
        return new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
    }

    /** Call in INIT with every motor idle and the robot at the route origin. */
    public static PinpointLocalizer openZeroed(HardwareMap hardwareMap, GoBildaPinpointDriver pinpoint) {
        // The constructor resets, waits for calibration, then takes one baseline reading.
        // Accept that deliberate jump even if the SDK cached a pose more than 5 m away.
        pinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.NONE);
        try {
            return new PinpointLocalizer(hardwareMap,
                    Constants.pinpointConfig(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU));
        } finally {
            pinpoint.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.LOCAL_TEST);
        }
    }

    /** The Pinpoint itself: the same device object Pedro's localizer reads, so its status is
     * refreshed every time the localizer updates. */
    public static GoBildaPinpointDriver device(HardwareMap hardwareMap) {
        return hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
    }

    /** The Pinpoint's own status (READY, CALIBRATING, FAULT_...), as text. Never throws. */
    public static String status(GoBildaPinpointDriver pinpoint) {
        if (pinpoint == null) return "NO PINPOINT";
        try { return String.valueOf(pinpoint.getDeviceStatus()); }
        catch (RuntimeException ex) { return "UNKNOWN (" + ex + ")"; }
    }

    /** Pose since the last zero. Before START it should sit near 0 while the robot is still. */
    public static void showPose(Telemetry telemetry, Pose pose) {
        boolean valid = Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
        telemetry.addData("Odometry x fwd / y left / heading", valid
                ? String.format(java.util.Locale.US, "%.1f in / %.1f in / %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()))
                : "NOT VALID - check the Pinpoint before START");
    }
}
