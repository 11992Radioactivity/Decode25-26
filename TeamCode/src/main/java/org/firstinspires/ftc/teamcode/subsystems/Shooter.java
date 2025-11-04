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

    // set as many motors as you want with one line of code
    private MotorGroup motors = new MotorGroup(
            new MotorEx("FlyWheelR") // right is leader because it doesn't have to be reversed
            //(new MotorEx("FlyWheelL")).reversed()
    );

    // - feedforward is good for general use but doesn't react fast
    // - pid is good for fast reaction but goes to 0 at setpoint which is bad for flywheel
    // solution = combine both for ultimate flywheel controller
    private ControlSystem control = ControlSystem.builder()
            .basicFF(0.000475, 0, 0) // power proportional to speed
            .velPid(0, 0) // power proportional to distance between current and set speed
            .build();

    private double ppr = 28; // pulses per revolution (28 for 6k rpm)
    private double rpmToPPS = ppr / 60; // (rpm / 60) * ppr

    public double targetSpeed = 4000;

    public Command off = new RunToVelocity(control, 0)
            .requires(this)
            .named("ShooterOff");
    public Command on = new RunToVelocity(control, targetSpeed * rpmToPPS)
            .requires(this)
            .named("ShooterOn");

    public void setSpeed(double rpm) {
        targetSpeed = rpm;

        on = new RunToVelocity(control, targetSpeed * rpmToPPS)
                .requires(this)
                .named("ShooterOn");
        on.schedule();
    }

    public double getCurrentSpeed() {
        return motors.getVelocity() / rpmToPPS;
    }

    @Override
    public void periodic() {
        motors.setPower(control.calculate(motors.getState()));
    }
}
