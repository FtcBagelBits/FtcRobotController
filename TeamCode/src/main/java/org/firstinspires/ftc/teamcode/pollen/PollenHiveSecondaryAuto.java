package org.firstinspires.ftc.teamcode.pollen;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="EXPERIMENTAL Hive Secondary",group="Pollen Hive",preselectTeleOp="PollenBot")
public final class PollenHiveSecondaryAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return true; }
}
