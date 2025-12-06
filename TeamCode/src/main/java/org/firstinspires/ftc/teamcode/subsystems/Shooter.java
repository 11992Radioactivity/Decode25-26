package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

// https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() {}

    // set as many motors as you want with one line of code
    private MotorGroup motors = new MotorGroup(
            (new MotorEx("FlyWheelR")).floatMode() // right is leader because it doesn't have to be reversed
            //(new MotorEx("FlyWheelL")).reversed()
    );

    VoltageSensor voltageSensor = ActiveOpMode.hardwareMap().voltageSensor.iterator().next();

    // - feedforward is good for general use but doesn't react fast
    // - pid is good for fast reaction but goes to 0 at setpoint which is bad for flywheel
    // solution = combine both for ultimate flywheel controller
    private ControlSystem control = ControlSystem.builder()
            .basicFF(0.0053 / voltageSensor.getVoltage(), 0, 1.7642 / voltageSensor.getVoltage()) // power proportional to speed
            .velPid(0.005) // power proportional to distance between current and set speed
            .build();

    private double ppr = 28; // pulses per revolution (28 for 6k rpm)
    private double rpmToPPS = ppr / 60; // (rpm / 60) * ppr

    public double targetSpeed = 2800;
    public double upValue = 0;

    public Command off = new RunToVelocity(control, 0)
            .requires(this)
            .named("ShooterOff");
    public Command on = new RunToVelocity(control, targetSpeed * rpmToPPS)
            .requires(this)
            .named("ShooterOn");

    public void setSpeed(double rpm) {
        rpm += upValue;
        if (Math.abs(rpm - targetSpeed) > 5) targetSpeed = rpm;

        on = new RunToVelocity(control, targetSpeed * rpmToPPS)
                .requires(this)
                .named("ShooterOn");
        on.schedule();
    }

    // https://www.desmos.com/calculator/rfsibijg0u
    private double getSpeedFromDistance(double distIn) {
        // constants in inches and radians
        double gravityIn = (9.81 * 1000) / 25.4;
        double shooterAngleRad = 60 * (Math.PI / 180);
        double goalHeightIn = 38.75;
        double shooterHeightIn = 16;

        // derived equation for magnitude of v0 in x = x0 + v0t + (1/2)at^2
        double numerator = gravityIn * distIn * distIn;
        double denominatorF1 = 2 * Math.pow(Math.cos(shooterAngleRad), 2);
        double denominatorF2 = distIn * Math.tan(shooterAngleRad) - (goalHeightIn - shooterHeightIn);
        double denominator = denominatorF1 * denominatorF2;
        double linearSpeed = Math.sqrt(numerator / denominator);

        return linearSpeed;
    }

    public void setSpeedFromLinearSpeed(double speedIn) {
        // linear speed to angular speed with a linear regression
        double a = 13.76347;
        double b = -93.76943;
        double rpm = a * speedIn + b;

        setSpeed(rpm);
    }

    public void setSpeedFromDistance(double distIn) {
        double linearSpeed = getSpeedFromDistance(distIn);
        setSpeedFromLinearSpeed(linearSpeed);
    }

    // set speed by subtracting velocity from goal position
    public void setSpeedCompensateRobotVelocity(Pose initialGoalPose, Pose robotPose, Vector robotVelocity) {
        Pose goalPose = initialGoalPose.copy();

        // iterate until distance from goal and speed converge
        for (int i = 0; i < 3; i++) {
            double dist = goalPose.distanceFrom(robotPose);
            double linearSpeed = getSpeedFromDistance(dist);
            double time = dist / (linearSpeed * Math.cos(Math.PI / 3));
            Vector newPose = goalPose.getAsVector().minus(robotVelocity.copy().times(time));
            goalPose = new Pose(newPose.getXComponent(), newPose.getYComponent());
        }

        double dist = goalPose.distanceFrom(robotPose);
        double linearSpeed = getSpeedFromDistance(dist);
        setSpeedFromLinearSpeed(linearSpeed);
    }

    public double getCurrentSpeed() {
        return motors.getVelocity() / rpmToPPS;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    @Override
    public void periodic() {
        if (Math.abs(control.getGoal().getVelocity() - motors.getVelocity()) < 0) motors.setPower(0);
        else motors.setPower(control.calculate(motors.getState()));
    }
}
