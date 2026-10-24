package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Option C: partner shoots first; our stationary shot, then right-forward parking. */
@Autonomous(name = "Pollen Supporting Shooter Auto", group = "Pollen", preselectTeleOp = "PollenBot")
public final class PollenSupportingShooter extends PollenShooterOpMode {
    @Override protected boolean supportingShooter() { return true; }
}
