package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class GateTest extends LinearOpMode {
    Servo gate;

    @Override
    public void runOpMode() {
        gate = hardwareMap.get(Servo.class, "Gate");
        long start = System.currentTimeMillis();
        double min = 0.415;
        double max = 0.4575;
        double cur = min;
        waitForStart();
        while (opModeIsActive()) {
            if (System.currentTimeMillis() - start >= 1000) {
                start = System.currentTimeMillis();
                if (cur == min) cur = max;
                else cur = min;
                gate.setPosition(cur);
            }
            telemetry.addData("open", cur == max);
            telemetry.update();
        }
    }
}
