package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.pedropathing.control.LowPassFilter;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.mathnstuff.InterpolatedLookupTable;

import java.util.function.DoubleSupplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
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

//    InterpolatedLookupTable vel_interplut = new InterpolatedLookupTable(
//            2200, 3050, // min and max to clamp to
//            40, 2200,
//            60, 2300,
//            70, 2500,
//            80, 2800,
//            100, 2850,
//            120, 3050
//    );
//
//    InterpolatedLookupTable hood_interplut = new InterpolatedLookupTable(
//            0.7, 0.25, // min and max to clamp to
//            60, 0.7,
//            80, 0.6,
//            100, 0.5,
//            120, 0.25
//    );

    InterpolatedLookupTable vel_interplut = new InterpolatedLookupTable(
            2125, 3125, // min and max to clamp to
            46.3, 2225,
            57.4, 2125,
            75.2, 2375,
            84, 2500,
            96.3, 2550,
            133.1, 2925,
            156.9, 3125
    );

    //96.3: 2550, 0.4
    //75.2: 2375, 0.5
    //57.4: 2125, 0.65
    //46.3: 2225, 0.7
    //133.1: 2925, 0.45
    //156.9: 3125, 0.5

    InterpolatedLookupTable hood_interplut = new InterpolatedLookupTable(
            0.7, 0.4, // min and max to clamp to
            46.3, 0.7,
            57.4, 0.65,
            75.2, 0.5,
            96.3, 0.4,
            133.1, 0.45,
            156.9, 0.5
    );

    private MotorEx leftMotor = new MotorEx("FlyWheelL").brakeMode();
    private MotorEx rightMotor = new MotorEx("FlyWheelR").reversed().brakeMode();

    // set as many motors as you want with one line of code
    private MotorGroup motors = new MotorGroup(
            leftMotor,
            rightMotor // right is leader because it doesn't have to be reversed
    );
    private ServoEx gate = new ServoEx("Gate");
    private ServoEx light = new ServoEx("Indicator");
    private ServoEx hood = new ServoEx("Hood");

    private double kV = 0.0047;
    private double kS = 1.5009;
    private double kPdef = 0.0025;
    private double kP = kPdef;
     // - feedforward is good for general use but doesn't react fast
    // - pid is good for fast reaction but goes to 0 at setpoint which is bad for flywheel
    // solution = combine both for ultimate flywheel controller
    private ControlSystem control = ControlSystem.builder()
            .basicFF(kV / 12, 0, kS / 12) // power proportional to speed
            .velPid(kP) // power proportional to distance between current and set speed
            .build();

    private final double ppr = 28; // pulses per revolution (28 for 6k rpm)
    private final double rpmToPPS = ppr / 60; // (rpm / 60) * ppr

    private final double gate_closed = 0.1;
    private final double gate_opened = 0.4;

    public double hood_pos = 0.7;
    public double targetSpeed = 2800;
    public LowPassFilter speedFilter = new LowPassFilter(0.9);
    public double upValue = 0;
    public double power_rate_per_sec = 1; // limit change in power to limit current
    public double last_timestamp = (double) System.currentTimeMillis();

    public Command off = new RunToVelocity(control, 0)
            .requires(this)
            .named("ShooterOff");
    public Command on = new RunToVelocity(control, targetSpeed * rpmToPPS)
            .requires(this)
            .named("ShooterOn");
    public Command openGate = new SetPosition(gate, gate_opened);
    public Command closeGate = new SetPosition(gate, gate_closed);

    public void closeGate() {
        gate.setPosition(gate_closed);
    }

    public void openGate() {
        gate.setPosition(gate_opened);
    }

    public double hood_deg_to_pos(double deg) {
        return deg * (7.0/180.0) - 65 * (7.0/180.0) + 0.7;
    }

    public double hood_pos_to_deg(double pos) {
        return pos * (180.0/7.0) - 0.7 * (180.0/7.0) + 65;
    }

    // when just maintaining inertia, minimize vibration with no proportional gain
    public void setPtoZero() {
        kP = 0;
        setVoltage(12);
    }

    public void setPtoDefault() {
        kP = kPdef;
        setVoltage(12);
    }

    public void setVoltage(double voltage) {
        control = ControlSystem.builder()
                .basicFF(kV / voltage, 0, kS / voltage)
                .velPid(kP)
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
    public double getLinearSpeedFromDistance(double distIn) {
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

    public double getTimeToGoal(double distIn) {
        double speed = getLinearSpeedFromDistance(distIn);
        double sin = Math.sin(Math.toRadians(60));
        double gravityIn = (9.81 * 1000) / 25.4;
        double n1 = speed * sin;
        double n2 = Math.sqrt(speed*speed * sin*sin - (2 * gravityIn * (-16)));
        double time = (n1 + n2) / gravityIn;
        return time;
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
            double linearSpeed = getLinearSpeedFromDistance(distIn);
            setSpeedFromLinearSpeed(linearSpeed);
        } else {
            setSpeed(vel_interplut.get(distIn));
        }

        hood_pos = hood_interplut.get(distIn);
    }

    public void setHoodPos(double pos) {
        hood_pos = Range.clip(pos, hood_interplut.get(999), hood_interplut.get(0));
    }

    // set speed by subtracting velocity from goal position
    public void setSpeedCompensateRobotVelocity(Pose initialGoalPose, Pose robotPose, Vector robotVelocity) {
        Pose goalPose = initialGoalPose.copy();

        // iterate until distance from goal and speed converge
        for (int i = 0; i < 3; i++) {
            double dist = goalPose.distanceFrom(robotPose);
            double linearSpeed = getLinearSpeedFromDistance(dist);
            double time = dist / (linearSpeed * Math.cos(Math.PI / 3));
            Vector newPose = goalPose.getAsVector().minus(robotVelocity.copy().times(time));
            goalPose = new Pose(newPose.getXComponent(), newPose.getYComponent());
        }

        double dist = goalPose.distanceFrom(robotPose);
        double linearSpeed = getLinearSpeedFromDistance(dist);
        setSpeedFromLinearSpeed(linearSpeed);
    }

    public boolean atTargetSpeed(double tolerance) {
        return Math.abs(getTargetSpeed() - getCurrentSpeed()) < tolerance;
    }

    public boolean aboveTargetSpeed(double tolerance) {
        return (getCurrentSpeed() - getTargetSpeed()) > tolerance;
    }

    public double getCurrentSpeed() {
        return speedFilter.getState() / rpmToPPS;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setLightGreen() {
        light.setPosition(0.5);
    }

    public void setLightRed() {
        light.setPosition(0.28);
    }

    public void lightOff() {
        light.setPosition(0);
    }

    @Override
    public void periodic() {
        double cur = (double) System.currentTimeMillis();
        double dt = (cur - last_timestamp) / 1000.0;
        last_timestamp = cur;

        speedFilter.update(motors.getVelocity(), 0);

        hood.setPosition(hood_pos);

        KineticState state = new KineticState(motors.getState().getPosition(), getCurrentSpeed() * rpmToPPS, motors.getState().getAcceleration());
        if (Math.abs(control.getGoal().getVelocity()) < 100) {
            motors.setPower(-0.01);
        } else {
            double target_effort = control.calculate(state);

            /*double avg_current = (leftMotor.getMotor().getCurrent(CurrentUnit.AMPS) + rightMotor.getMotor().getCurrent(CurrentUnit.AMPS)) / 2;
            if (avg_current > 6) {
                target_effort = 0.05;
            }*/

            motors.setPower(target_effort);

            double vel_diff = getTargetSpeed() - getCurrentSpeed();
            double hood_offset = (-2.5 * vel_diff) / 125.0;
            double new_hood_pos = hood_deg_to_pos(hood_pos_to_deg(hood_pos) - hood_offset);
            new_hood_pos = Range.clip(new_hood_pos, hood_interplut.get(999), hood_interplut.get(0));
            hood.setPosition(new_hood_pos);

            /*double cur_effort = motors.getPower();
            if (Math.abs(target_effort - cur_effort) > power_rate_per_sec * dt && Math.abs(control.getGoal().getVelocity() - motors.getVelocity()) > 100) {
                motors.setPower(cur_effort + power_rate_per_sec * dt * Math.signum(target_effort - cur_effort));
            } else {
                motors.setPower(target_effort);
            }*/
        }

        if (getCurrentSpeed() < 2000) {
            //light.setPosition(0);
            //hood.setPosition(hood_interplut.get(0));
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
