package org.firstinspires.ftc.teamcode.pollen;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Beta Test Primary Shooter + Garden", group = "2 Beta Test", preselectTeleOp = "PollenBot")
public final class PollenHivePrimaryAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return false; }
}
