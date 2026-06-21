package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Red Far 12", preselectTeleOp = "ManualTeleOp")
public class RedFar12 extends AbstractFar12or15 {
    public RedFar12() {
        super(false, false);
    }
}
