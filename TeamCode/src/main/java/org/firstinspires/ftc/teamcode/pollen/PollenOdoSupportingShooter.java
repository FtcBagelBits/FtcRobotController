package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** The partner shoots first: wait, shoot, then drive each leg to a measured end point with
 * odometry. (The timed version, EMERGENCY Timed Supporting Shooter, is only for when the odometry
 * sensor has failed.) */
@Autonomous(name = "Emergency Odo Supporting Shooter", group = "3 Emergency", preselectTeleOp = "PollenBot")
public final class PollenOdoSupportingShooter extends PollenShooterOpMode {
    @Override protected boolean supportingShooter() { return true; }
    @Override protected boolean odometry() { return true; }
}
