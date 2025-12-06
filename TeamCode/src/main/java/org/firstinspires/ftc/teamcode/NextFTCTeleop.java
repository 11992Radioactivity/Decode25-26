package org.firstinspires.ftc.teamcode;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.*;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class NextFTCTeleop extends NextFTCOpMode {
    public NextFTCTeleop() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE)
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left_motor").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right_motor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left_motor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right_motor").brakeMode();
    private final JoinedTelemetry telemetryM = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    private final GamepadManager panelsGamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
    private final GamepadManager panelsGamepad2 = PanelsGamepad.INSTANCE.getSecondManager();
    private final MotorEx intake = new MotorEx("FlyWheelL");
    private final CRServoEx transferL = new CRServoEx("TransferL");
    private final CRServoEx transferR = new CRServoEx("TransferR");

    private final boolean onBlue = true;
    private boolean autoAim = false;
    private double targetHeading = 0;
    private boolean activeTurning = false;
    private ElapsedTime turnTimer = new ElapsedTime();

    private Pose goalPose;

    private final double autoAimGain = 1.0 / 45.0; // 1 / (point to slow down at after reaching)

    @Override
    public void onStartButtonPressed() {
        Shooter.INSTANCE.off.schedule();

        PedroComponent.follower().setStartingPose(new Pose(72, 72));
        if (onBlue) goalPose = new Pose(4, 132);
        else goalPose = new Pose(144, 136);

        Drawing.init();

        GamepadEx gp1 = new GamepadEx(() -> panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()));
        GamepadEx gp2 = new GamepadEx(() -> panelsGamepad2.asCombinedFTCGamepad(ActiveOpMode.gamepad2()));

        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                gp1.leftStickY().negate().deadZone(0.2),
                () -> { // use triggers to strafe
                    if (panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).left_trigger > 0.2) {
                        return (double) -panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).left_trigger;
                    } else if (panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).right_trigger > 0.2) {
                        return (double) panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).right_trigger;
                    }
                    return 0.0;
                },
                () -> { // auto aim when in shooting mode
                    double stickX = panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).right_stick_x;
                    //Pose target = goalPose.copy().minus(PedroComponent.follower().getPose());
                    //double angle = Math.atan2(target.getY(), target.getX());
                    //telemetryM.addData("auto aim angle", Math.toDegrees(angle));
                    //if (!autoAim) {
                    if (Math.abs(stickX) > 0.4) {
                        activeTurning = true;
                        targetHeading = PedroComponent.follower().getHeading();
                        turnTimer.reset();
                        return (stickX * Math.abs(stickX)) / 2.0;
                    } else if (activeTurning && turnTimer.seconds() > 0.05) {
                        targetHeading = PedroComponent.follower().getHeading();
                        activeTurning = false;
                    }
                    double error = Math.toDegrees(targetHeading - PedroComponent.follower().getHeading());
                    if (error > 180) {
                        error -= 360;
                    } else if (error < -180) {
                        error += 360;
                    }
                    return -0.03 * error;
                    //}
                    //double error = angle - PedroComponent.follower().getHeading();
                    //return 0.0 * Math.toDegrees(error);
                }//,
                //new FieldCentric(() -> Angle.fromRad(PedroComponent.follower().getHeading()))
        );
        driverControlled.schedule();

        gp1.b().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    intake.setPower(-1);
                })
                .whenBecomesFalse(() -> {
                    intake.setPower(0);
                });

        gp1.x().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    transferL.setPower(1);
                    transferR.setPower(-1);
                })
                .whenBecomesFalse(() -> {
                    transferL.setPower(0);
                    transferR.setPower(0);
                });

        gp1.a().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    autoAim = true;
                    double dist = PedroComponent.follower().getPose().distanceFrom(goalPose);
                    Shooter.INSTANCE.setSpeedFromDistance(dist);
                })
                .whenBecomesFalse(() -> {
                    autoAim = false;
                    Shooter.INSTANCE.off.schedule();
                });

        /*gp1.dpadUp().whenBecomesTrue(() -> {
            if (autoAim) Shooter.INSTANCE.setSpeed(Shooter.INSTANCE.targetSpeed + 25);
        });

        gp1.dpadDown().whenBecomesTrue(() -> {
            if (autoAim) Shooter.INSTANCE.setSpeed(Shooter.INSTANCE.targetSpeed - 25);
        });*/
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        double dist = PedroComponent.follower().getPose().distanceFrom(goalPose);

        // TODO: Test shooting while moving
        if (autoAim) { // update shooter speed outside of just when you press the button
            Shooter.INSTANCE.setSpeedFromDistance(dist);

            targetHeading = goalPose.getAsVector().minus(PedroComponent.follower().getPose().getAsVector()).getTheta();
            // scheduling new speed command every frame is fine because
            // command manager just cancels old commands that are doing the same thing
        }

        telemetryM.addData("angle vel", PedroComponent.follower().getAngularVelocity());
        telemetryM.addData("target heading", targetHeading);
        telemetryM.addData("shooter  vel", Shooter.INSTANCE.getCurrentSpeed());
        telemetryM.addData("shooter target", Shooter.INSTANCE.getTargetSpeed());
        telemetryM.addData("distance from goal", dist);
        telemetryM.addData("position", PedroComponent.follower().getPose());
        telemetryM.addData("velocity", PedroComponent.follower().getVelocity());

        Drawing.drawDebug(PedroComponent.follower());
        telemetryM.update();
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.off.schedule();
        BindingManager.reset();
    }
}