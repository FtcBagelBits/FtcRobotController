package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Same as Pollen ROBOT Shoot + Park, but each leg drives to a measured end point with odometry. */
@Autonomous(name = "Odo ROBOT Shoot + Park", group = "Pollen Odometry", preselectTeleOp = "PollenBot")
public final class PollenOdoRobotShoot extends PollenShooterOpMode {
    @Override protected boolean supportingShooter() { return false; }
    @Override protected boolean odometry() { return true; }
}
