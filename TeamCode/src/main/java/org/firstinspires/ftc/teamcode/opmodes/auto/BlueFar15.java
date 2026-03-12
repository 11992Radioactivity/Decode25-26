package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Far 15", preselectTeleOp = "ManualTeleOp")
public class BlueFar15 extends AbstractFar12or15 {
    public BlueFar15() {
        super(true, true);
    }
}
