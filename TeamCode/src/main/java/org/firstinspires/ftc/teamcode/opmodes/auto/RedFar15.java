package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Far 15", preselectTeleOp = "ManualTeleOp")
public class RedFar15 extends AbstractFar12or15 {
    public RedFar15() {
        super(false, true);
    }
}
