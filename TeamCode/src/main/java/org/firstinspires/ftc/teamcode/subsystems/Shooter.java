package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter() {}

    private MotorGroup motors = new MotorGroup(
            new MotorEx("FlyWheelR"), // right is leader because it doesn't have to be reversed
            (new MotorEx("FlyWheelL")).reversed()
    );

    private ControlSystem control = ControlSystem.builder()
            .velPid(0.0065, 0.018) // i gets most of the way and p gives a little nudge
            .build();

    private final double ppr = 28; // pulses per revolution (28 for 6k rpm)
    private final double rpmToPPS = ppr / 60; // (rpm / 60) * ppr

    private double targetSpeed = 2400 * rpmToPPS;

    public Command off = new RunToVelocity(control, 0).requires(this).named("ShooterOff");
    public Command on = new RunToVelocity(control, targetSpeed).requires(this).named("ShooterOn");

    public void setTargetSpeed(double rpm) {
        targetSpeed = rpm * rpmToPPS;

        on = new RunToVelocity(control, targetSpeed).requires(this).named("ShooterOn");
    }

    public double getCurrentSpeed() {
        return motors.getVelocity() / rpmToPPS;
    }

    @Override
    public void periodic() {
        motors.setPower(control.calculate(motors.getState()));
    }
}
