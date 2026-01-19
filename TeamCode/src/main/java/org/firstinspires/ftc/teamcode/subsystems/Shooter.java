package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.mathnstuff.InterpolatedLookupTable;

import java.util.function.DoubleSupplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

// https://medium.com/@vikramaditya.nishant/programming-a-decode-shooter-4ab114dac01f
public class Shooter implements Subsystem {
    public final static Shooter INSTANCE = new Shooter();
    private Shooter() {}

    // use equation VS. use InterpLUT to compute velocity
    private boolean usePhysics = false;

    InterpolatedLookupTable interplut = new InterpolatedLookupTable(
            2250, 3700, // min and max to clamp to
            47.9, 2250,
            57.4, 2350,
            68.7, 2450,
            82.6, 2675,
            93.5, 2750,
            98.5, 2750,
            111.1, 2975,
            120.9, 3225,
            138.3, 3425,
            148.3, 3700
    );

    // set as many motors as you want with one line of code
    private MotorGroup motors = new MotorGroup(
            (new MotorEx("FlyWheelR")).floatMode(), // right is leader because it doesn't have to be reversed
            (new MotorEx("FlyWheelL")).reversed().floatMode()
    );
    private ServoEx gate = new ServoEx("Gate");
    private ServoEx light = new ServoEx("Indicator");

    // - feedforward is good for general use but doesn't react fast
    // - pid is good for fast reaction but goes to 0 at setpoint which is bad for flywheel
    // solution = combine both for ultimate flywheel controller
    private ControlSystem control = ControlSystem.builder()
            .basicFF(0.0054 / 12, 0, 2.945 / 12) // power proportional to speed
            .velPid(0.01) // power proportional to distance between current and set speed
            .build();

    private double ppr = 28; // pulses per revolution (28 for 6k rpm)
    private double rpmToPPS = ppr / 60; // (rpm / 60) * ppr

    private double gate_closed = 0.0;
    private double gate_opened = 0.4;

    public double targetSpeed = 2800;
    public double upValue = 0;
    public double power_rate_per_sec = 3; // limit change in power to limit current
    public double last_timestamp = (double) System.currentTimeMillis();

    public Command off = new RunToVelocity(control, 0)
            .requires(this)
            .named("ShooterOff");
    public Command on = new RunToVelocity(control, targetSpeed * rpmToPPS)
            .requires(this)
            .named("ShooterOn");
    public Command openGate = new SetPosition(gate, gate_opened);
    public Command closeGate = new SetPosition(gate, gate_closed);

    public void setVoltage(double voltage) {
        control = ControlSystem.builder()
                .basicFF(0.00506 / voltage, 0, 2.8222 / voltage)
                .velPid(0.01)
                .build();
    }

    // weird thing only for auto
    public Command onFromDistSupplier(DoubleSupplier dist, double overridetrigger, double overrideamt) {
        return new Command() {
                @Override
                public boolean isDone() {
                    double d = dist.getAsDouble();
                    if (d > overridetrigger) d = overrideamt;
                    setSpeedFromDistance(d);
                    return true;
                }
            };
    }

    public Command setSpeedCommand(double rpm) {
        return new Command() {
            @Override
            public boolean isDone() {
                setSpeed(rpm);
                return true;
            }
        };
    }

    public void setSpeed(double rpm) {
        //rpm += upValue;
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

    private void setSpeedFromLinearSpeed(double speedIn) {
        // linear speed to angular speed with a linear regression
        double a = 13.76347;
        double b = -93.76943;
        double rpm = a * speedIn + b;

        setSpeed(rpm);
    }

    public void setSpeedFromDistance(double distIn) {
        if (usePhysics) {
            double linearSpeed = getSpeedFromDistance(distIn);
            setSpeedFromLinearSpeed(linearSpeed);
        } else {
            setSpeed(interplut.get(distIn));
        }
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

    public boolean atTargetSpeed(double tolerance) {
        return Math.abs(getTargetSpeed() - getCurrentSpeed()) < tolerance;
    }

    public double getCurrentSpeed() {
        return motors.getVelocity() / rpmToPPS;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    @Override
    public void periodic() {
        double cur = (double) System.currentTimeMillis();
        double dt = (cur - last_timestamp) / 1000.0;
        last_timestamp = cur;

        if (Math.abs(control.getGoal().getVelocity()) < 100) {
            motors.setPower(0);
        } else {
            double target_effort = control.calculate(motors.getState());
            double cur_effort = motors.getPower();
            if (Math.abs(target_effort - cur_effort) > power_rate_per_sec * dt && Math.abs(control.getGoal().getVelocity() - motors.getVelocity()) > 100) {
                motors.setPower(cur_effort + power_rate_per_sec * dt * Math.signum(target_effort - cur_effort));
            } else {
                motors.setPower(target_effort);
            }
        }

        if (getCurrentSpeed() < 1500) {
            light.setPosition(0);
        } else {
            if (atTargetSpeed(50)) {
                light.setPosition(0.5); // Green
            } else if (atTargetSpeed(500)) {
                light.setPosition(0.39); // Yellow
            } else {
                light.setPosition(0.28); // Red
            }
        }
    }
}
