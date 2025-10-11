package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import java.lang.reflect.Array;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp
public class FlyWheelTest extends LinearOpMode {
    private DcMotorEx flywheel;
    private final double ticksPerRev = 145.1;//28;

    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "FlyWheel");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double targetSpeed = 600;
        double lastEnc = flywheel.getCurrentPosition();
        double lastTime = System.currentTimeMillis();
        double integral = 0;
        ArrayList<Double> lastVels = new ArrayList<>();
        double start = lastTime;

        // rev hd hex : p (0.00005) i (0.00001) f (0.1)
        // gobilda 1150 : p (0.005) i (0.0001) f (0.2)

        while (opModeIsActive()) {
            double curEnc = flywheel.getCurrentPosition();
            double curTime = System.currentTimeMillis();
            double dt = (curTime - lastTime) / 1000;
            double velocity = (curEnc - lastEnc) / dt;

            lastEnc = curEnc;
            lastTime = curTime;
            lastVels.add(velocity);

            if (lastVels.size() > 10) {
                lastVels.remove(0);
            }

            if (curTime - start > 3000) {
                targetSpeed = 1100;
            }

            if (curTime - start > 6000) {
                targetSpeed = 600;
                start = curTime;
            }

            double sum = 0;
            for (int i = 0; i < lastVels.size(); i++) {
                sum += lastVels.get(i);
            }
            double avgVelocity = sum / 10;
            double rpm = (avgVelocity / ticksPerRev) * 60;

            double error = targetSpeed - rpm;
            integral += error;
            double power = (error * 0.0075) + (integral * 0.0005) + 0.2;
            power = Math.max(0.2, power);
            flywheel.setPower(power);

            telemetry.addData("velocity (tick/sec)", avgVelocity);
            telemetry.addData("velocity (revs/min)", rpm);
            telemetry.update();
        }
    }
}