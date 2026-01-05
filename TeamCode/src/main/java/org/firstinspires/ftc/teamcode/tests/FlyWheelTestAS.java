package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import dev.nextftc.control.feedback.PIDCoefficients;
//import dev.nextftc.control.feedback.PIDController;


@TeleOp
@Disabled
public class FlyWheelTestAS extends LinearOpMode {
    private DcMotorEx flywheelL;
    private DcMotorEx flywheelR;
    //private PIDController leftController = new PIDController(new PIDCoefficients(0.0075, 0.0005, 0));
    //private PIDController rightController = new PIDController(new PIDCoefficients(0.0075, 0.0005, 0));
    private final double ticksPerRev = 145.1;//28;

    public void runOpMode() {
        flywheelL = hardwareMap.get(DcMotorEx.class, "FlyWheelL");
        flywheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelR = hardwareMap.get(DcMotorEx.class, "FlyWheelR");
        flywheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        /*leftController.reset();
        rightController.reset();

        leftController.calculate()*/

        double targetSpeed = 6000;
        /*double lastEnc = flywheel.getCurrentPosition();
        double lastTime = System.currentTimeMillis();
        double integral = 0;
        ArrayList<Double> lastVels = new ArrayList<>();
        double start = lastTime;*/

        // rev hd hex : p (0.00005) i (0.00001) f (0.1)
        // gobilda 1150 : p (0.005) i (0.0001) f (0.2)

        while (opModeIsActive()) {
            /*double curEnc = flywheel.getCurrentPosition();
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
            power = Math.max(0.2, power);*/

            flywheelL.setVelocity(-targetSpeed * (28.0 / 60.0));
            flywheelR.setVelocity(targetSpeed * (28.0 / 60.0));
            //flywheelL.setPower(-0.1);
            //flywheelR.setPower(0.1);

            //telemetry.addData("velocity (tick/sec)", avgVelocity);
            //telemetry.addData("velocity (revs/min)", rpm);
            telemetry.update();
        }
    }
}