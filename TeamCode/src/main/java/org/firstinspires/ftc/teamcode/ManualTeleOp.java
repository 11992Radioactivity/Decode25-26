package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.control.KalmanFilter;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.*;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagCamera;
import org.firstinspires.ftc.teamcode.subsystems.DataStorage;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.UtilFunctions;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Rotation2d;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@TeleOp
public class ManualTeleOp extends NextFTCOpMode {
    public ManualTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
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
    private DriverControlledCommand driverControlled;
    private AprilTagCamera camera;

    // non constant variables
    private boolean onBlue = true;
    private boolean autoAim = false;
    private double targetHeading = 0;
    private boolean activeTurning = false;
    private ElapsedTime turnTimer = new ElapsedTime();
    private double DIST_ADJUST = 6;
    private boolean intake_on = false;
    private boolean transfer_on = false;

    // constant variables
    private final boolean log_server = true; //TODO: MUST TURN OFF DURING COMPETITION
    private final boolean use_triggers_for_sideways = false;
    private final boolean relocalize_on_shoot = false;
    private Pose goalPose;
    private final double autoAimGain = 1.0 / 45.0; // 1 / (point to slow down at after reaching)

    @Override
    public void onInit() {
        Shooter.INSTANCE.off.schedule();

        SimpleDateFormat fmt = new SimpleDateFormat("MM_dd_yyyy_HH_mm", Locale.US);
        String filename = fmt.format(new Date());
        Logger.addDataReceiver(new RLOGWriter(filename + ".rlog"));

        // TODO: Figure out FTCDashboard protocol so it shows up on AdvantageScope
        // because rn putting 192.168.43.1 is just waiting forever
        if (log_server) {
            RLOGServer server = new RLOGServer();
            server.start();
            Logger.addDataReceiver(server);
        }

        Logger.recordMetadata("OpMode Name", "ManualTeleOp");
        Logger.recordMetadata("OpMode Type", "TeleOp");

        Logger.start();
        Logger.periodicAfterUser(0, 0);
    }

    @Override
    public void onStartButtonPressed() {
        telemetryM = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        panelsGamepad1 = PanelsGamepad.INSTANCE.getFirstManager();
        panelsGamepad2 = PanelsGamepad.INSTANCE.getSecondManager();
        camera = new AprilTagCamera(hardwareMap);

        Shooter.INSTANCE.off.schedule();

        onBlue = DataStorage.INSTANCE.onBlue;
        PedroComponent.follower().setStartingPose(DataStorage.INSTANCE.teleopStartPose);
        targetHeading = DataStorage.INSTANCE.teleopStartPose.getHeading();
        if (onBlue) goalPose = new Pose(4, 140);
        else goalPose = new Pose(140, 140);

        Drawing.init();

        GamepadEx gp1 = new GamepadEx(() -> panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()));
        GamepadEx gp2 = new GamepadEx(() -> panelsGamepad2.asCombinedFTCGamepad(ActiveOpMode.gamepad2()));

        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                gp1.leftStickY().negate().deadZone(0.3),
                () -> { // use triggers to strafe
                    if (!use_triggers_for_sideways) return gp1.leftStickX().deadZone(0.3).get() * (onBlue ? -1 : 0);
                    if (panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).left_trigger > 0.3) {
                        return (double) -panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).left_trigger;
                    } else if (panelsGamepad1.asCombinedFTCGamepad(ActiveOpMode.gamepad1()).right_trigger > 0.3) {
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
                },
                new FieldCentric(() -> Angle.fromRad(PedroComponent.follower().getHeading() + Math.PI * (onBlue ? 1 : 0)))
        );
        driverControlled.schedule();

        Command shootCommand = new SequentialGroup(
                Shooter.INSTANCE.openGate,
                new Delay(0.3),
                new SetPower(intake, -1),
                new SetPower(transfer, -1)
        );

        gp1.b().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    if (autoAim) {
                        shootCommand.schedule();

                        intake_on = true;
                        transfer_on = true;
                    } else {
                        intake.setPower(-1);
                        transfer.setPower(0);
                        Shooter.INSTANCE.closeGate.schedule();

                        intake_on = true;
                        transfer_on = false;
                    }
                })
                .whenBecomesFalse(() -> {
                    intake.setPower(0);
                    transfer.setPower(0);
                    Shooter.INSTANCE.closeGate.schedule();

                    intake_on = false;
                    transfer_on = false;
                });

        // on press, override driver control and go to park,
        // on off reschedule driver control
        gp1.x().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    Pose basePose = new Pose(
                            onBlue ? 105 : 38.5,
                            33.5
                    );
                    PathChain toBase = PedroComponent.follower()
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(PedroComponent.follower().getPose(), basePose)
                            )
                            .setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), 0)
                            .build();
                    Command moveCommand = new FollowPath(toBase);
                    moveCommand.schedule();
                })
                .whenBecomesFalse(() -> {
                    driverControlled.schedule();
                });

        gp1.a().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    autoAim = true;
                    double dist = PedroComponent.follower().getPose().distanceFrom(goalPose);
                    Shooter.INSTANCE.setSpeedFromDistance(dist + DIST_ADJUST);
                })
                .whenBecomesFalse(() -> {
                    autoAim = false;
                    Shooter.INSTANCE.off.schedule();
                    transfer.setPower(0);
                    Shooter.INSTANCE.closeGate.schedule();
                    transfer_on = false;
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
        double beforeUserStart = Logger.getTimestamp();
        Logger.periodicBeforeUser();
        double beforeUserEnd = Logger.getTimestamp();

        BindingManager.update();
        double dist = PedroComponent.follower().getPose().distanceFrom(goalPose);

        Pose robotPoseCamera = new Pose();
        boolean cameraFoundTag = false;

        // TODO: Test shooting while moving
        if (autoAim) {
            // update shooter speed continuously
            // - scheduling new speed command every frame is fine because
            //   command manager just cancels old commands that are doing the same thing
            Shooter.INSTANCE.setSpeedFromDistance(dist + DIST_ADJUST);

            targetHeading = goalPose.getAsVector().minus(PedroComponent.follower().getPose().getAsVector()).getTheta();

            // prioritize powering shooter over others if both are on
            UtilFunctions.currentLimitMotor(intake, (intake_on ? -1 : 0));
            UtilFunctions.currentLimitMotor(transfer, (transfer_on ? -1 : 0));

            camera.update();
            List<AprilTagDetection> tags = camera.getDetections();

            if (!tags.isEmpty()) {
                AprilTagDetection tag = null;
                for (AprilTagDetection d : tags) {
                    if (d.id == 20 || d.id == 24) { // don't track from obelisk
                        tag = d;
                        break;
                    }
                }

                // found tag
                if (tag != null) {
                    robotPoseCamera = camera.getRobotPoseFromTag(tag);
                    cameraFoundTag = true;
                }

                // relocalize if not moving so position isn't skewed
                if (relocalize_on_shoot && PedroComponent.follower().getVelocity().getMagnitude() < 0.1 && Math.abs(targetHeading - PedroComponent.follower().getHeading()) < Math.toRadians(20)) {
                    PedroComponent.follower().setPose(robotPoseCamera);
                }
            }
        } else {
            Shooter.INSTANCE.closeGate.schedule();
        }

        telemetryM.addData("target heading", targetHeading);
        telemetryM.addData("shooter vel", Shooter.INSTANCE.getCurrentSpeed());
        telemetryM.addData("shooter target", Shooter.INSTANCE.getTargetSpeed());
        telemetryM.addData("distance from goal", dist);
        telemetryM.addData("position", PedroComponent.follower().getPose());
        telemetryM.addData("camera found tag?", cameraFoundTag);
        telemetryM.addData("camera pose", robotPoseCamera);
        telemetryM.addData("velocity", PedroComponent.follower().getVelocity());

        Logger.recordOutput("OpMode/TargetHeading", targetHeading);
        Logger.recordOutput("OpMode/ShooterVelocity", Shooter.INSTANCE.getCurrentSpeed());
        Logger.recordOutput("OpMode/ShooterTarget", Shooter.INSTANCE.getTargetSpeed());
        Logger.recordOutput("OpMode/DistanceFromGoal", dist);
        Logger.recordOutput("OpMode/CameraFoundTag", cameraFoundTag);

        Pose pose = PedroComponent.follower().getPose().getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose2d poseMeters = new Pose2d(pose.getX() / 39.37, pose.getY() / 39.37, new Rotation2d(pose.getHeading()));
        Logger.recordOutput("OpMode/PoseMeters", Pose2d.struct, poseMeters);

        Pose cameraPose = robotPoseCamera.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Pose2d cameraPoseMeters = new Pose2d(cameraPose.getX() / 39.37, cameraPose.getY() / 39.37, new Rotation2d(cameraPose.getHeading()));
        Logger.recordOutput("OpMode/CameraPoseMeters", Pose2d.struct, cameraPoseMeters);

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