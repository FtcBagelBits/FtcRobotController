package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Isolated practice using the same controller, route and settings as the hive autos. */
@Autonomous(name = "Calibration Primary Shot Only", group = "4 Calibration")
public final class PollenHiveShotPractice extends PollenHiveOpMode {
    @Override protected boolean secondary() { return false; }
    @Override protected PollenHiveAuto.Run run() { return PollenHiveAuto.Run.SHOT_ONLY; }
}
