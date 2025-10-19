package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class NextFTCTeleop extends NextFTCOpMode {
    public NextFTCTeleop() {
        addComponents(
                //new SubsystemComponent(Lift.INSTANCE, Claw.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("front_left_motor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("front_right_motor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left_motor").brakeMode().reversed();
    private final MotorEx backRightMotor = new MotorEx("back_right_motor").brakeMode();

    @Override
    public void onStartButtonPressed() {
        /*frontLeftMotor.setPower(0.1);
        frontRightMotor.setPower(0.1);
        backLeftMotor.setPower(0.1);
        backRightMotor.setPower(0.1);*/
        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                () -> Double.valueOf(-ActiveOpMode.gamepad1().left_stick_y),
                () -> Double.valueOf(ActiveOpMode.gamepad1().left_stick_x),
                () -> Double.valueOf(ActiveOpMode.gamepad1().right_stick_x)
        );
        driverControlled.schedule();

        /*Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(Lift.INSTANCE.toHigh)
                .whenBecomesFalse(Claw.INSTANCE.open);

        Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(
                        Claw.INSTANCE.close.then(Lift.INSTANCE.toHigh)
                );

        Gamepads.gamepad2().leftBumper().whenBecomesTrue(
                Claw.INSTANCE.open.and(Lift.INSTANCE.toLow)
        );*/
    }
}