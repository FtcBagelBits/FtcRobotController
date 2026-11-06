package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Same as Pollen Supporting Shooter Auto, but each leg drives to a measured end point with odometry. */
@Autonomous(name = "Odo Supporting Shooter Auto", group = "Pollen Odometry", preselectTeleOp = "PollenBot")
public final class PollenOdoSupportingShooter extends PollenShooterOpMode {
    @Override protected boolean supportingShooter() { return true; }
    @Override protected boolean odometry() { return true; }
}
