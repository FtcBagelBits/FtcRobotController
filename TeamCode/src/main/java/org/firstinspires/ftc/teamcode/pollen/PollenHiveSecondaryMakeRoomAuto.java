package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** Play 2, "make room": the partner is the hero and will shoot from our side of the hive.
 * Same start, camera gate and shot as EXPERIMENTAL Hive Secondary. After the shot it drives
 * straight to the corner where the scoring-end wall meets the side wall (as far out of the
 * hero's way as our half allows), waits there, then parks in the outer slot when the set
 * seconds are left before the 27 s stop (default 2 s). Shares the secondary's saved settings. */
@Autonomous(name = "EXPERIMENTAL Hive Secondary MAKE ROOM", group = "Pollen Hive", preselectTeleOp = "PollenBot")
public final class PollenHiveSecondaryMakeRoomAuto extends PollenHiveOpMode {
    @Override protected boolean secondary() { return true; }
    @Override protected boolean makeRoom() { return true; }
}
