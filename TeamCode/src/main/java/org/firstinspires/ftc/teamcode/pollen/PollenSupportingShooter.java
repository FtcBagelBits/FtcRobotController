package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** EMERGENCY ONLY: the timed version of Odo Supporting Shooter Auto, for a match where the
 * odometry sensor (Pinpoint) is not working. Partner shoots first; our stationary shot, then
 * right-forward parking, with every leg driven for a set time. */
@Autonomous(name = "EMERGENCY Timed Supporting Shooter", group = "Pollen Emergency (timed)", preselectTeleOp = "PollenBot")
public final class PollenSupportingShooter extends PollenShooterOpMode {
    @Override protected boolean supportingShooter() { return true; }
}
