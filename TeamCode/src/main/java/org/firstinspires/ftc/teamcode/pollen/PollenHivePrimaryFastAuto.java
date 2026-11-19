package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Primary garden plan tuned for cycle time, with its own saved profile: a closer garden entry,
 * 2.5 s feeds and one nudge back onto the shot spot. Why: docs/research/cycle-time.md. */
@Autonomous(name = "Beta Primary Shooter - Fast", group = "2 Beta Test", preselectTeleOp = "PollenBot")
public final class PollenHivePrimaryFastAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return false; }
    @Override protected boolean fast() { return true; }
}
