package org.firstinspires.ftc.teamcode.pollen;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name = "Secondary Shooter + Flower", group = "1 Match", preselectTeleOp = "PollenBot")
public final class PollenHiveSecondaryAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return true; }
}
