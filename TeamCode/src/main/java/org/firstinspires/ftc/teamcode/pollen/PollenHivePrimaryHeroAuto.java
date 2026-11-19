package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Primary garden plan with its own profile for testing the hero role. */
@Autonomous(name = "Beta Primary Shooter - Hero", group = "2 Beta Test", preselectTeleOp = "PollenBot")
public final class PollenHivePrimaryHeroAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return false; }
    @Override protected boolean hero() { return true; }
}
