package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.hardware.impl.ServoEx;

@TeleOp
public class RGBLightTest extends OpMode {
    Servo light;
    ElapsedTime timer;

    public double RED = 0.28, YELLOW = 0.39, GREEN = 0.5;
    double color = 0;
    double last = 0;
    double dir = 1;

    @Override
    public void init() {
        timer = new ElapsedTime();
        last = 0;
        light = hardwareMap.get(Servo.class, "Indicator");
    }

    @Override
    public void loop() {
        double dt = timer.seconds() - last;
        last = timer.seconds();

        color += 0.3 * dt * dir;

        if (color > 0.72) {
            dir = -1;
        } else if (color < RED) {
            dir = 1;
        }

        light.setPosition(color);
    }
}
