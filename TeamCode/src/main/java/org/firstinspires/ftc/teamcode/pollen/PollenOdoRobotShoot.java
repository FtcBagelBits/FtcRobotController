package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Our main auto: shoot, then drive each leg to a measured end point with odometry. (The timed
 * version, EMERGENCY Timed Shoot + Park, is only for when the odometry sensor has failed.) */
@Autonomous(name = "Odo ROBOT Shoot + Park", group = "Pollen Odometry", preselectTeleOp = "PollenBot")
public final class PollenOdoRobotShoot extends PollenShooterOpMode {
    @Override protected boolean supportingShooter() { return false; }
    @Override protected boolean odometry() { return true; }
}
