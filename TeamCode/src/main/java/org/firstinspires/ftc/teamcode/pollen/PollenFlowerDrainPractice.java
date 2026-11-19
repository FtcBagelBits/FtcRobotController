package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Calibration Flower Drain", group = "4 Calibration")
public final class PollenFlowerDrainPractice extends PollenCollectionPractice {
    @Override protected PollenCollectionSequence.Kind kind() { return PollenCollectionSequence.Kind.FLOWER; }
}
