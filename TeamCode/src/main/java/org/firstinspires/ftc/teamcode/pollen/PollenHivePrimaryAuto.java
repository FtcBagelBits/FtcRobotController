package org.firstinspires.ftc.teamcode.pollen;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="EXPERIMENTAL Hive Primary + Garden",group="Pollen Hive",preselectTeleOp="PollenBot")
public final class PollenHivePrimaryAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return false; }
}
