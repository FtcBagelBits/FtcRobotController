package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** EMERGENCY ONLY: the timed version of Odo ROBOT Shoot + Park, for a match where the odometry
 * sensor (Pinpoint) is not working. It drives each leg for a set time and cannot tell where the
 * robot is, so it parks less accurately. Stationary shot, then the selected departure. */
@Autonomous(name = "EMERGENCY Timed Shoot + Park", group = "Pollen Emergency (timed)", preselectTeleOp = "PollenBot")
public final class PollenRobotShoot extends PollenShooterOpMode {
    @Override protected boolean supportingShooter() { return false; }
}
