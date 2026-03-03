package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mathnstuff.DataStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Autonomous(name = "Blue Close 15 Auto", preselectTeleOp = "ManualTeleOp")
@Disabled
public class BlueClose15Auto extends NextFTCOpMode {
    public BlueClose15Auto() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public static class Paths {

        private Follower follower = PedroComponent.follower();
        public PathChain PreloadShoot;
        public PathChain GrabSpike1;
        public PathChain Shoot1;
        public PathChain OpenGate1;
        public PathChain GrabGate1;
        public PathChain Shoot2;
        public PathChain OpenGate2;
        public PathChain GrabGate2;
        public PathChain Shoot3;
        public PathChain GrabSpike2;
        public PathChain Shoot4;

        public Pose shootPose = new Pose(48, 96);

        public Paths() {
            PreloadShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(20.300, 122.600),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(135))

                    .build();

            GrabSpike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(65.000, 55.000),
                                    new Pose(10.000, 60.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.000, 60.000),
                                    new Pose(65.000, 55.000),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            OpenGate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(65.000, 55.000),
                                    new Pose(12.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(145))

                    .build();

            GrabGate1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 61.000),
                                    new Pose(10.000, 56.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(145))

                    .build();

            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.000, 56.000),
                                    new Pose(65.000, 55.000),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))

                    .build();

            OpenGate2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(65.000, 55.000),
                                    new Pose(12.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(145))

                    .build();

            GrabGate2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 61.000),

                                    new Pose(10.000, 56.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(145))

                    .build();

            Shoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(10.000, 56.000),
                                    new Pose(65.000, 55.000),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))

                    .build();

            GrabSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.000, 96.000),
                                    new Pose(48.550, 80.725),
                                    new Pose(18.000, 82.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Shoot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.000, 82.000),

                                    new Pose(60.000, 110.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(152.5))

                    .build();
        }
    }

    private ElapsedTime timer = new ElapsedTime();

    private boolean intake_on = false;
    private MotorEx intake = new MotorEx("Intake");
    private MotorEx transfer = new MotorEx("Transfer");
    private Command intakeOn = new Command() {
        @Override
        public boolean isDone() {
            intake_on = true;
            intake.setPower(1);
            return true;
        }
    };
    private Command intakeOff = new Command() {
        @Override
        public boolean isDone() {
            intake_on = false;
            intake.setPower(0);
            return true;
        }
    };
    private Command transferOn = new SetPower(transfer, -1);
    private Command transferOff = new SetPower(transfer, 0);

    private Command auto;
    private double curTime = 0;
    private boolean done = false;

    @Override
    public void onInit() {
        Shooter.INSTANCE.setSpeed(0);
    }

    @Override
    public void onStartButtonPressed() {
        Drawing.init();

        PedroComponent.follower().setStartingPose(new Pose(20.300, 122.600, Math.toRadians(140)));

        Command shoot = new SequentialGroup(
                Shooter.INSTANCE.openGate,
                new Delay(0.3),
                intakeOn,
                transferOn,
                new Delay(0.5),
                intakeOff,
                transferOff,
                Shooter.INSTANCE.closeGate
        );

        Paths paths = new Paths();

        auto = new SequentialGroup(
                Shooter.INSTANCE.setSpeedCommand(2725),
                new FollowPath(paths.PreloadShoot),
                new Delay(0.5),
                shoot, //shoot
                intakeOn,
                new FollowPath(paths.GrabSpike1),
                new FollowPath(paths.Shoot1),
                intakeOff,
                new Delay(0.25),
                shoot, //shoot
                intakeOn,
                new FollowPath(paths.OpenGate1),
                new Delay(0.25),
                new FollowPath(paths.GrabGate1),
                new Delay(0.25),
                new FollowPath(paths.Shoot2),
                intakeOff,
                new Delay(0.25),
                shoot, //shoot
                intakeOn,
                new FollowPath(paths.OpenGate2),
                new Delay(0.25),
                new FollowPath(paths.GrabGate2),
                new Delay(0.25),
                new FollowPath(paths.Shoot2),
                intakeOff,
                new Delay(0.25),
                shoot, //shoot
                /* 1 more cycle for 18 if there's time
                intakeOn,
                new FollowPath(paths.GrabGate2),
                new Delay(1),
                intakeOff,
                new FollowPath(paths.Shoot3),
                shoot, //shoot
                 */
                intakeOn,
                new FollowPath(paths.GrabSpike2),
                new Delay(0.25),
                new FollowPath(paths.Shoot4),
                intakeOff,
                shoot,
                new Command() {
                    @Override
                    public boolean isDone() {
                        done = true;
                        return true;
                    }
                }
        );

        auto.schedule();

        timer.reset();
    }

    @Override
    public void onUpdate() {
        if (!done) {
            DataStorage.INSTANCE.onBlue = true;
            DataStorage.INSTANCE.teleopStartPose = PedroComponent.follower().getPose();
            curTime = timer.time();
        } else {
            Shooter.INSTANCE.setSpeed(0);
        }
        telemetryM.debug("time", curTime);
        telemetryM.debug("position", PedroComponent.follower().getPose());
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());

        Drawing.drawDebug(PedroComponent.follower());
        telemetryM.update();
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.setSpeed(0);
    }
}
