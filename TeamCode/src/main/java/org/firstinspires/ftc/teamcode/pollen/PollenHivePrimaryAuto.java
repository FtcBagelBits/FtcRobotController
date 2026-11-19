package org.firstinspires.ftc.teamcode.pollen;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Primary Shooter + Garden", group = "1 Match", preselectTeleOp = "PollenBot")
public final class PollenHivePrimaryAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return false; }
}
