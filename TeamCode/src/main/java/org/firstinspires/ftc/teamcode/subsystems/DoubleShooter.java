package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class DoubleShooter implements Subsystem {
    public static final DoubleShooter INSTANCE = new DoubleShooter();
    private DoubleShooter() {}

    private MotorEx left = new MotorEx("FlyWheelL");
    private MotorEx right = new MotorEx("FlyWheelR");
    private MotorGroup motors = new MotorGroup(left, right);
    
    private double leftSpeed = 0;
    private double rightSpeed = 0;

    private ControlSystem controlL = ControlSystem.builder()
            .velPid(0.01)
            .basicFF(0, 0, 0.1)
            .build();

    private ControlSystem controlR = ControlSystem.builder()
            .velPid(0.01)
            .basicFF(0, 0, 0.1)
            .build();

    private final double ppr = 28; // pulses per revolution (28 for 6k rpm)
    private final double rpmToPPS = ppr / 60; // (rpm / 60) * ppr

    //far zone: 2650, close zone: 2250
    private double targetSpeed = 2250 * rpmToPPS;

    public Command off = new ParallelGroup(
            new RunToVelocity(controlL, 0),
            new RunToVelocity(controlR, 0)
    ).requires(this).named("ShooterOff");
    public Command on = new ParallelGroup(
            new RunToVelocity(controlL, -targetSpeed),
            new RunToVelocity(controlR, targetSpeed)
    ).requires(this).named("ShooterOn");

    public void setTargetSpeed(double rpm) {
        targetSpeed = rpm * rpmToPPS;

        on = new ParallelGroup(
                new RunToVelocity(controlL, -targetSpeed),
                new RunToVelocity(controlR, targetSpeed)
        ).requires(this).named("ShooterOn");
    }

    @Override
    public void periodic() {
        KineticState leftState = left.getState();
        KineticState rightState = right.getState();

        leftSpeed = leftState.getVelocity() / rpmToPPS;
        rightSpeed = rightState.getVelocity() / rpmToPPS;

        left.setPower(controlL.calculate(leftState));
        right.setPower(controlR.calculate(rightState));
    }

    public double getLeftSpeed() {
        return leftSpeed;
    }

    public double getRightSpeed() {
        return rightSpeed;
    }
}
