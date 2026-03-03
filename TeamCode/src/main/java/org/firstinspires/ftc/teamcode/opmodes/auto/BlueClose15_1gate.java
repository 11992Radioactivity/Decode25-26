package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Close 15 1 gate", preselectTeleOp = "ManualTeleOp")
public class BlueClose15_1gate extends AbstractClose15 {
    public BlueClose15_1gate() {
        super(true, false);
    }
}
