package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Lift implements Subsystem {
    public final static Lift INSTANCE = new Lift();
    private Lift() {}

    private MotorGroup motors = new MotorGroup(
            (new MotorEx("LiftR")),
            (new MotorEx("LiftL")).reversed()
    );

    private ControlSystem control = ControlSystem.builder()
            .posPid(0.1)
            .elevatorFF(0.05)
            .build();

    // ((ticks / ppr) * 8) / 25.4
    private double ppr = 145.1;
    private double inchesToTicks = (ppr * 25.4) / 8;

    public Command liftUp = new RunToPosition(control, 8 * inchesToTicks);
    public Command liftDown = new RunToPosition(control, 0);

    @Override
    public void periodic() {
        double power = control.calculate(motors.getState());
        // cap power off at 0.1 so it doesnt snap if wrong direction
        power = Math.signum(power) * Math.max(Math.abs(power), 0.1);
        motors.setPower(power);
    }
}
