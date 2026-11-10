package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * EXPERIMENTAL: the primary shooter's auto with an angled shot. Not yet tried on a field.
 *
 * It starts in the same place as Odo ROBOT Shoot + Park (front flat against the end wall, back
 * to the hive, lined up with the hive cell that starts up) and parks in the same slot. The difference is where it
 * shoots from: it first drives off the wall and to the left (legs 1 and 2), turns in place to
 * point the launcher at the hive, shoots, and turns back. Leg 3 then goes to the parking slot.
 *
 * Because it drives before it shoots, it needs working odometry to shoot at all. If INIT shows a
 * Pinpoint problem, use Odo ROBOT Shoot + Park instead: that one shoots first.
 *
 * The route, its INIT rows and the aim angle are in PollenShootTuning (look for "angled").
 */
@Autonomous(name = "Angled Primary Shooter", group = "Pollen Experimental", preselectTeleOp = "PollenBot")
public final class PollenAngledPrimaryShooter extends PollenShooterOpMode {
    @Override protected boolean supportingShooter() { return false; }
    @Override protected boolean odometry() { return true; }
    @Override protected boolean angledShot() { return true; }
}
