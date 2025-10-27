package org.firstinspires.ftc.teamcode;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.*;
import org.firstinspires.ftc.teamcode.subsystems.DoubleShooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.HolonomicMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class NextFTCTeleop extends NextFTCOpMode {
    public NextFTCTeleop() {
        addComponents(
                //new SubsystemComponent(DoubleShooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left_motor").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right_motor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left_motor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right_motor").brakeMode();
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final GamepadManager panelsGamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
    private final GamepadManager panelsGamepad2 = PanelsGamepad.INSTANCE.getSecondManager();

    @Override
    public void onStartButtonPressed() {
        Drawing.init();

        GamepadEx gp1 = new GamepadEx(() -> panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()));
        GamepadEx gp2 = new GamepadEx(() -> panelsGamepad2.asCombinedFTCGamepad(ActiveOpMode.gamepad2()));

        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                gp1.leftStickY().negate(),
                gp1.leftStickX(),
                gp1.rightStickX().map(x -> x/2)
                //new FieldCentric(() -> Angle.fromRad(PedroComponent.follower().getHeading()))
        );
        driverControlled.schedule();

        /*
        gp1.dpadUp()
                .whenBecomesTrue(DoubleShooter.INSTANCE.on);
        gp1.dpadDown()
                .whenBecomesTrue(DoubleShooter.INSTANCE.off);
        */

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

    @Override
    public void onUpdate() {
        telemetryM.debug("stickY", (double) -panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).left_stick_y);
        telemetryM.debug("stickX", (double) panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).left_stick_x);
        telemetryM.debug("position", PedroComponent.follower().getPose());
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());

        Drawing.drawDebug(PedroComponent.follower());
        telemetryM.update();
    }
}