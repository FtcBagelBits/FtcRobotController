package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Secondary Shooter + Flower, shot from an angle (v41). From the secondary start it drives one
 * straight leg to a spot off the wall and to the right (toward the flower side) with the flywheel
 * spinning up, turns to aim at our cell (PollenFieldGeometry.aimDegrees, allowing for the cell's
 * middle being 2.7 in right of the start), waits for our cell UP (the partner's tip), shoots,
 * drains the flower, comes back to the same spot, turns, shoots again without a camera wait, and
 * parks in the outer slot. Otherwise the same as Secondary Shooter + Flower. Own saved profile
 * (secondary-angled). */
@Autonomous(name = "Beta Secondary Shooter - Angled", group = "2 Beta Test", preselectTeleOp = "PollenBot")
public final class PollenHiveSecondaryAngledAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return true; }
    @Override protected boolean angled() { return true; }
}
