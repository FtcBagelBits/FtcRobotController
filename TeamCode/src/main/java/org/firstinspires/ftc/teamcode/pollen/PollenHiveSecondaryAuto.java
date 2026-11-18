package org.firstinspires.ftc.teamcode.pollen;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Beta Test Secondary Shooter", group = "2 Beta Test", preselectTeleOp = "PollenBot")
public final class PollenHiveSecondaryAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return true; }
}
