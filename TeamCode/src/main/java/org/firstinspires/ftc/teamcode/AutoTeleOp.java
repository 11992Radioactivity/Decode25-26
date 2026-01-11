package org.firstinspires.ftc.teamcode;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.*;
import org.firstinspires.ftc.teamcode.mathnstuff.DataStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

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
import dev.nextftc.hardware.impl.MotorEx;

// state machine auto operator
// inspired by frc 1690 2024 auto shooter
@TeleOp
@Disabled
public class AutoTeleOp extends NextFTCOpMode {
    public AutoTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Shooter.INSTANCE)//, Lift.INSTANCE)
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left_motor").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right_motor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left_motor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right_motor").brakeMode();
    private final JoinedTelemetry telemetryM = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
    private final GamepadManager panelsGamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
    private final GamepadManager panelsGamepad2 = PanelsGamepad.INSTANCE.getSecondManager();
    private final MotorEx intake = new MotorEx("Intake");

    private final boolean onBlue = DataStorage.INSTANCE.onBlue;
    private boolean autoAim = false;
    private double targetHeading = 0;
    private boolean activeTurning = false;
    private ElapsedTime turnTimer = new ElapsedTime();

    //TODO: MUST TURN OFF DURING COMPETITION
    private boolean log_server = true;

    private Pose goalPose;

    private final double autoAimGain = 1.0 / 45.0; // 1 / (point to slow down at after reaching)

    @Override
    public void onInit() {
        SimpleDateFormat fmt = new SimpleDateFormat("MM_dd_yyyy_HH_mm", Locale.US);
        String filename = fmt.format(new Date());
        Logger.addDataReceiver(new RLOGWriter("/sdcard/FIRST/" + filename + ".rlog"));

        if (log_server) {
            Logger.addDataReceiver(new RLOGServer());
        }

        Logger.recordMetadata("OpMode Name", "TeleOp");
        Logger.recordMetadata("OpMode Type", "TeleOp");

        Logger.start();
        Logger.periodicAfterUser(0, 0);
    }

    @Override
    public void onStartButtonPressed() {
        Shooter.INSTANCE.off.schedule();

        PedroComponent.follower().setStartingPose(DataStorage.INSTANCE.teleopStartPose);
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
                gp1.leftStickY().negate().deadZone(0.3),
                () -> { // use triggers to strafe
                    if (panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).left_trigger > 0.3) {
                        return (double) -panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).left_trigger;
                    } else if (panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).right_trigger > 0.3) {
                        return (double) panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).right_trigger;
                    }
                    return 0.0;
                },
                () -> { // auto aim when in shooting mode
                    double stickX = panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).right_stick_x;
                    if (Math.abs(stickX) > 0.4) {
                        activeTurning = true;
                        targetHeading = PedroComponent.follower().getHeading();
                        turnTimer.reset();
                        return (stickX * Math.abs(stickX)) / 2.0;
                    } else if (activeTurning && turnTimer.seconds() > 0.1) {
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
                }//,
                //new FieldCentric(() -> Angle.fromRad(PedroComponent.follower().getHeading()))
        );
        driverControlled.schedule();

        /*gp1.x().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    Lift.INSTANCE.liftUp.schedule();
                })
                .whenBecomesFalse(() -> {
                    Lift.INSTANCE.liftDown.schedule();
                });*/
    }

    // visualization: https://www.desmos.com/calculator/gbyxebiyvz
    private boolean inShootingZone(Pose pose) {
        double x = pose.getX();
        double y = pose.getY();

        double close_right = x;
        double close_left = -x + 144;
        double far_left = x - 48;
        double far_right = -x + 96;

        if (x < 48) {
            return y > close_left;
        } else if (x < 72) {
            return y > close_left || y < far_left;
        } else if (x < 96) {
            return y > close_right || y < far_right;
        } else {
            return y > close_right;
        }
    }

    @Override
    public void onUpdate() {
        double beforeUserStart = Logger.getTimestamp();
        Logger.periodicBeforeUser();
        double beforeUserEnd = Logger.getTimestamp();

        BindingManager.update();
        Pose pose = PedroComponent.follower().getPose();
        double dist = pose.distanceFrom(goalPose);
        double angle_from_goal = goalPose.getAsVector().minus(pose.getAsVector()).getTheta();
        // motor stall current is 9.2A, and when intake is full it is around 7
        double intake_current = intake.getMotor().getCurrent(CurrentUnit.AMPS);
        boolean intake_stalled = intake_current > 7;

        // TODO: using timers and state machine is probably easier
        if (autoAim && intake_stalled && inShootingZone(pose) && Shooter.INSTANCE.atTargetSpeed(50) && AngleUnit.normalizeRadians(angle_from_goal - pose.getHeading()) < Math.toRadians(5)) {
            intake.setPower(-1);
            Shooter.INSTANCE.openGate.schedule();
        } else if (autoAim && !intake_stalled) {
            autoAim = false;
            Shooter.INSTANCE.closeGate.schedule();
            Shooter.INSTANCE.off.schedule();
            intake.setPower(-1);
        } else if (intake_stalled && inShootingZone(pose) && AngleUnit.normalizeRadians(angle_from_goal - pose.getHeading()) < Math.toRadians(30)) {
            autoAim = true;
        } else if (!inShootingZone(pose)) {
            autoAim = false;
            Shooter.INSTANCE.off.schedule();
            Shooter.INSTANCE.closeGate.schedule();
            intake.setPower(-1);
        }

        // TODO: Test shooting while moving
        if (autoAim) { // update shooter speed outside of just when you press the button
            Shooter.INSTANCE.setSpeedFromDistance(dist);

            targetHeading = angle_from_goal;
            // scheduling new speed command every frame is fine because
            // command manager just cancels old commands that are doing the same thing
        }

        telemetryM.addData("intake current", intake_current);
        telemetryM.addData("angle vel", PedroComponent.follower().getAngularVelocity());
        telemetryM.addData("target heading", targetHeading);
        telemetryM.addData("shooter  vel", Shooter.INSTANCE.getCurrentSpeed());
        telemetryM.addData("shooter target", Shooter.INSTANCE.getTargetSpeed());
        telemetryM.addData("distance from goal", dist);
        telemetryM.addData("position", pose);
        telemetryM.addData("velocity", PedroComponent.follower().getVelocity());

        Logger.recordOutput("Intake Current", intake_current);
        Logger.recordOutput("OpMode/TargetHeading", targetHeading);
        Logger.recordOutput("OpMode/ShooterVelocity", Shooter.INSTANCE.getCurrentSpeed());
        Logger.recordOutput("OpMode/ShooterTarget", Shooter.INSTANCE.getTargetSpeed());
        Logger.recordOutput("OpMode/DistanceFromGoal", dist);

        Pose2d poseMeters = new Pose2d(pose.getX() / 39.37, pose.getY() / 39.37, new Rotation2d(pose.getHeading()));
        Logger.recordOutput("OpMode/PoseMeters", Pose2d.struct, poseMeters);

        Drawing.drawDebug(PedroComponent.follower());
        telemetryM.update();

        double afterUserStart = Logger.getTimestamp();
        Logger.periodicAfterUser(
                afterUserStart - beforeUserEnd,
                beforeUserEnd - beforeUserStart
        );
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.off.schedule();
        BindingManager.reset();
        Logger.end();
    }
}