package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Isolated practice using the same controller, route and settings as the hive autos. */
@Autonomous(name = "Calibration Secondary Shot Only", group = "4 Calibration")
public final class PollenHiveSecondaryShotPractice extends PollenHiveOpMode {
    @Override protected boolean secondary() { return true; }
    @Override protected PollenHiveAuto.Run run() { return PollenHiveAuto.Run.SHOT_ONLY; }
}
