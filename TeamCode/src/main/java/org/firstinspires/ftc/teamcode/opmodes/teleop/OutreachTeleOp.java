package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

// no fancy aiming fluff or field oriented drive
@TeleOp
public class OutreachTeleOp extends NextFTCOpMode {
    public OutreachTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }

    // constant systems/components
    private final MotorEx frontLeftMotor = new MotorEx("front_left_motor").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right_motor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left_motor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right_motor").brakeMode();
    private final MotorEx intake = new MotorEx("Intake").brakeMode();
    private final MotorEx transfer = new MotorEx("Transfer").brakeMode();
    private JoinedTelemetry telemetryM;
    private GamepadManager panelsGamepad1;
    private GamepadManager panelsGamepad2;

    // non constant variables
    private boolean shooting = false;
    private boolean intake_on = false;
    private boolean shot = false;
    private double voltage = 12;

    private Command shootCommand = new SequentialGroup(
            Shooter.INSTANCE.openGate,
            new Delay(0.3),
            new SetPower(intake, 1),
            new SetPower(transfer, -1)
    );

    @Override
    public void onInit() {
        Shooter.INSTANCE.setSpeed(0);
    }

    @Override
    public void onStartButtonPressed() {
        telemetryM = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        panelsGamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
        panelsGamepad2 = PanelsGamepad.INSTANCE.getSecondManager();

        GamepadEx gp1 = new GamepadEx(() -> panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()));
        GamepadEx gp2 = new GamepadEx(() -> panelsGamepad2.asCombinedFTCGamepad(ActiveOpMode.gamepad2()));

        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                gp1.leftStickY().negate().deadZone(0.3),
                gp1.leftStickX().deadZone(0.3),
                gp1.rightStickX().deadZone(0.3)
        );
        driverControlled.schedule();

        gp1.b().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    intake.setPower(1);
                    transfer.setPower(0);
                    Shooter.INSTANCE.closeGate();

                    intake_on = true;
                })
                .whenBecomesFalse(() -> {
                    intake.setPower(0);
                    transfer.setPower(0);
                    Shooter.INSTANCE.closeGate();

                    intake_on = false;
                });

        gp1.a().whenBecomesTrue(() -> {
            shooting = true;
        });

        gp1.a().whenBecomesFalse(() -> {
            shot = false;
            shooting = false;
            transfer.setPower(0);
            intake.setPower(0);
            Shooter.INSTANCE.closeGate();
            intake_on = false;
        });
    }

    @Override
    public void onUpdate() {
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        Shooter.INSTANCE.setVoltage(voltage);

        BindingManager.update();

        if (shooting) {
            Shooter.INSTANCE.setSpeed(2800);

            if (!shot && Shooter.INSTANCE.aboveTargetSpeed(25)) {
                shot = true;
                shootCommand.schedule();

                intake_on = true;
            }
        } else {
            Shooter.INSTANCE.setPtoZero();
            Shooter.INSTANCE.closeGate();
            Shooter.INSTANCE.setSpeed(1200);
        }

        double intake_current = intake.getMotor().getCurrent(CurrentUnit.AMPS);
        if (intake_on && intake_current > 5 && intake.getPower() == 1) {
            intake.setPower(0);
        } else if (intake_on && intake_current < 5) {
            intake.setPower(1);
        }

        telemetryM.addData("intake current", intake_current);
        telemetryM.addData("shooter vel", Shooter.INSTANCE.getCurrentSpeed());
        telemetryM.addData("shooter target", Shooter.INSTANCE.getTargetSpeed());
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.off.schedule();
        BindingManager.reset();
    }
}