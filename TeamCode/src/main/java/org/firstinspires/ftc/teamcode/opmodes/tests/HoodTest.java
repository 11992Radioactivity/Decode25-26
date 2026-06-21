package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HoodTest extends LinearOpMode {
    Servo hood;

    @Override
    public void runOpMode() {
        hood = hardwareMap.get(Servo.class, "Hood");
        long start = System.currentTimeMillis();
        double min = 0.4;
        double max = 0.7;
        double cur = min;
        waitForStart();
        while (opModeIsActive()) {
            if (System.currentTimeMillis() - start >= 1000) {
                start = System.currentTimeMillis();
                if (cur == min) cur = max;
                else cur = min;
                hood.setPosition(cur);
            }
            telemetry.addData("down", cur == max);
            telemetry.update();
        }
    }
}
