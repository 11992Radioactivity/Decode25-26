package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Close 15 2 gate", preselectTeleOp = "ManualTeleOp")
public class RedClose15_2gate extends AbstractClose15 {
    public RedClose15_2gate() {
        super(false, true);
    }
}
