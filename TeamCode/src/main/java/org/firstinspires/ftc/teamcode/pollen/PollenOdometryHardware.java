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
                    + "\" in the active configuration; odometry autos cannot run (use the timed Pollen autos)");
        }
        return new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
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
