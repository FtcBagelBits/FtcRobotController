package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Calibration Garden Pickup", group = "4 Calibration")
public final class PollenGardenPickupPractice extends PollenCollectionPractice {
    @Override protected PollenCollectionSequence.Kind kind() { return PollenCollectionSequence.Kind.GARDEN; }
}
