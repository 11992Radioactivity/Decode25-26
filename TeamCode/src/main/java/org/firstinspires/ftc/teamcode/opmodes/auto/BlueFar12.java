package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Far 12", preselectTeleOp = "ManualTeleOp")
public class BlueFar12 extends AbstractFar12or15 {
    public BlueFar12() {
        super(true, false);
    }
}
