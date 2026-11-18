package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PRACTICE Flower Drain", group = "Pollen Collection")
public final class PollenFlowerDrainPractice extends PollenCollectionPractice {
    @Override protected PollenCollectionSequence.Kind kind() { return PollenCollectionSequence.Kind.FLOWER; }
}
