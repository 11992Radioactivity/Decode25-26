package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.hardware.impl.MotorEx;

// functions used in multiple classes but couldn't
// figure out which subsystems to put them in
public class UtilFunctions {
    // continuously decreases power until motor is below current limit
    // - must call every loop
    // - current unit is in AMPS (usual motor stall is 9A, struggle at 6A)
    public static void currentLimitMotor(MotorEx motor, double requested_power, double max_current) {
        double current = motor.getMotor().getCurrent(CurrentUnit.AMPS);
        if (current > max_current) {
            motor.setPower(motor.getPower() * 0.9);
        } else {
            motor.setPower(requested_power);
        }
    }

    // default to 6A based on intake stall testing
    public static void currentLimitMotor(MotorEx motor, double requested_power) {
        currentLimitMotor(motor, requested_power, 6);
    }
}
