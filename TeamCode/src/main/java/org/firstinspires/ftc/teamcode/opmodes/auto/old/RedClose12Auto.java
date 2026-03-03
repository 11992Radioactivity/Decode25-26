package org.firstinspires.ftc.teamcode.opmodes.auto.old;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.mathnstuff.DataStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Autonomous(name = "Red Close 12 Auto", preselectTeleOp = "ManualTeleOp")
public class RedClose12Auto extends NextFTCOpMode {
    public RedClose12Auto() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public static class Paths {

        private Follower follower = PedroComponent.follower();
        public PathChain Shoot1;
        public PathChain Grab1Init;
        public PathChain Grab1Grab;
        public PathChain GatePush;
        public PathChain Shoot2;
        public PathChain Grab2Init;
        public PathChain Grab2Grab;
        public PathChain Shoot3;
        public PathChain Grab3Init;
        public PathChain Grab3Grab;
        public PathChain Shoot4;
        public PathChain MoveOffLaunchLine;

        public Pose shootPose = new Pose(48.000, 96.000).mirror();

        public Paths() {
            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.300, 122.600).mirror(), shootPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140 - 90), Math.toRadians(140 - 90))
                    .build();

            Grab1Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPose, new Pose(50.000, 84.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135 - 90), Math.toRadians(0))
                    .build();

            Grab1Grab = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 84.000).mirror(), new Pose(16.000, 84.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GatePush = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.000, 84.000).mirror(),
                                    new Pose(35.229, 84.119).mirror(),
                                    new Pose(13.000, 74.000).mirror()
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(13.000, 74.000).mirror(), shootPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140 - 90))
                    .build();

            Grab2Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPose, new Pose(50.000, 60.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135 - 90), Math.toRadians(0))
                    .build();

            Grab2Grab = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 60.000).mirror(), new Pose(10.000, 60.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Shoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(10.000, 60.000).mirror(),
                                    new Pose(41.394, 73.321).mirror(),
                                    shootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140 - 90))
                    .build();

            Grab3Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPose, new Pose(50.000, 37.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135 - 90), Math.toRadians(0))
                    .build();

            Grab3Grab = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 37.000).mirror(), new Pose(10.000, 37.000).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Shoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 37.000).mirror(), new Pose(60.000, 110.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180 - 152.5))
                    .build();

            MoveOffLaunchLine = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPose, new Pose(30.000, 70.00).mirror())
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135 - 90), Math.toRadians(270))
                    .build();
        }
    }

    private ElapsedTime timer = new ElapsedTime();

    private Pose goalPose = new Pose(4, 132).mirror();

    private MotorEx intake = new MotorEx("Intake");
    private MotorEx transfer = new MotorEx("Transfer");
    private Command intakeOn = new SetPower(intake, 1);
    private Command intakeOff = new SetPower(intake, 0);
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

        PedroComponent.follower().setStartingPose(new Pose(123.700, 122.600, Math.toRadians(40)));

        Command shoot = new SequentialGroup(
                Shooter.INSTANCE.openGate,
                new Delay(0.3),
                intakeOn,
                transferOn,
                new Delay(0.5),
                intakeOff,
                transferOff,
                new ParallelGroup(Shooter.INSTANCE.closeGate, intakeOff)
        );

        Paths paths = new Paths();

        auto = new SequentialGroup(
                Shooter.INSTANCE.setSpeedCommand(2725),
                new FollowPath(paths.Shoot1),
                new Delay(0.5),
                shoot, //shoot
                new FollowPath(paths.Grab1Init),
                intakeOn,
                new FollowPath(paths.Grab1Grab, true, 0.7),
                new FollowPath(paths.GatePush, true, 0.7),
                new Delay(0.5),
                new FollowPath(paths.Shoot2),
                intakeOff,
                shoot, //shoot
                new FollowPath(paths.Grab2Init),
                intakeOn,
                new FollowPath(paths.Grab2Grab, true, 0.7),
                new FollowPath(paths.Shoot3),
                intakeOff,
                shoot, //shoot
                new FollowPath(paths.Grab3Init),
                intakeOn,
                new FollowPath(paths.Grab3Grab, true, 0.7),
                new FollowPath(paths.Shoot4),
                intakeOff,
                shoot, //shoot
                new Command() {
                    @Override
                    public boolean isDone() {
                        done = true;
                        return true;
                    }
                }
                //new FollowPath(paths.MoveOffLaunchLine),
        );

        auto.schedule();

        timer.reset();
    }

    @Override
    public void onUpdate() {
        if (!done) {
            curTime = timer.seconds();
            DataStorage.INSTANCE.onBlue = false;
            DataStorage.INSTANCE.teleopStartPose = PedroComponent.follower().getPose();
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
