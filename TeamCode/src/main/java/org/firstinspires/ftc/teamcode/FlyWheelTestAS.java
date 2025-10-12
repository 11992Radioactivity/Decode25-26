package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//import dev.nextftc.control.feedback.PIDCoefficients;
//import dev.nextftc.control.feedback.PIDController;


@TeleOp
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

        double targetSpeed = 1100;
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

            flywheelL.setVelocity(-targetSpeed * 60 * 360, AngleUnit.DEGREES);
            flywheelR.setVelocity(targetSpeed * 60 * 360, AngleUnit.DEGREES);

            //telemetry.addData("velocity (tick/sec)", avgVelocity);
            //telemetry.addData("velocity (revs/min)", rpm);
            telemetry.update();
        }
    }
}