package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    private MotorEx motor = new MotorEx("intake");

    private ControlSystem control = ControlSystem.builder()
            .velPid(0.005, 0.0001)
            .basicFF(0.2)
            .build();

    private final double ppr = 145.1; // pulses per revolution (28 for 6k rpm)
    private final double rpmToPPS = ppr / 60; // (rpm / 60) * ppr

    private double targetSpeed = 1150 * rpmToPPS;

    public Command off = new RunToVelocity(control, 0).requires(this).named("IntakeOff");
    public Command on = new RunToVelocity(control, targetSpeed).requires(this).named("IntakeOn");

    public void setTargetSpeed(double rpm) {
        targetSpeed = rpm * rpmToPPS;

        on = new RunToVelocity(control, targetSpeed).requires(this).named("IntakeOn");
    }

    @Override
    public void periodic() {
        motor.setPower(control.calculate(motor.getState()));
    }
}
