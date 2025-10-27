package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class SingleShooter implements Subsystem {
    public static final SingleShooter INSTANCE = new SingleShooter();
    private SingleShooter() {}

    private MotorEx motor = new MotorEx("FlyWheelL");

    private ControlSystem control = ControlSystem.builder()
            .velPid(0.00005, 0.00001)
            .basicFF(0.1)
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

    @Override
    public void periodic() {
        motor.setPower(control.calculate(motor.getState()));
    }
}
