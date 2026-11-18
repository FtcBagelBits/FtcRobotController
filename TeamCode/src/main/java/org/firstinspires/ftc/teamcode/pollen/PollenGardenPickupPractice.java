package org.firstinspires.ftc.teamcode.pollen;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PRACTICE Garden Pickup", group = "Pollen Collection")
public final class PollenGardenPickupPractice extends PollenCollectionPractice {
    @Override protected PollenCollectionSequence.Kind kind() { return PollenCollectionSequence.Kind.GARDEN; }
}
