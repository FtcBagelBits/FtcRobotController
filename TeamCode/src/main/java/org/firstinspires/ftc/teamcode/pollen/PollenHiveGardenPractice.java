package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Isolated practice using the same controller, route and settings as the hive autos. */
@Autonomous(name = "TEST Hive garden + return", group = "Pollen Hive")
public final class PollenHiveGardenPractice extends PollenHiveOpMode {
    @Override protected boolean secondary() { return false; }
    @Override protected PollenHiveAuto.Run run() { return PollenHiveAuto.Run.GARDEN_RETURN; }
}
