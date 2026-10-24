package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Option B: stationary shot followed by the selected robot-relative departure. */
@Autonomous(name = "Pollen ROBOT Shoot + Park", group = "Pollen", preselectTeleOp = "PollenBot")
public final class PollenRobotShoot extends PollenShooterOpMode {
    @Override protected boolean supportingShooter() { return false; }
}
