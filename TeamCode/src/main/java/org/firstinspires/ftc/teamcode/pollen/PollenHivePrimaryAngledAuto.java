package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Primary garden plan that shoots both volleys from a spot left of our hive cell (toward the
 * garden) and off the wall, turned right to aim at the cell. The spot is two INIT rows (inches
 * left, inches away from the wall); the aim angle follows from them (PollenFieldGeometry.aimDegrees).
 * It drives one straight leg there with the flywheel spinning up, turns, shoots, drains the garden,
 * returns to the same spot, turns, shoots, and parks. Own saved profile (primary-angled). */
@Autonomous(name = "Beta Primary Shooter - Angled Left", group = "2 Beta Test", preselectTeleOp = "PollenBot")
public final class PollenHivePrimaryAngledAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return false; }
    @Override protected boolean angled() { return true; }
}
