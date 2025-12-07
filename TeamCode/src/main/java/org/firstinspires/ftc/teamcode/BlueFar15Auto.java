package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.DataStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Autonomous(name = "Blue Far 15 Auto", preselectTeleOp = "TeleOp")
public class BlueFar15Auto extends NextFTCOpMode {
    public BlueFar15Auto() {
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
        public PathChain Shoot2;
        public PathChain Grab2Init;
        public PathChain Grab2Grab;
        public PathChain Shoot3;
        public PathChain Grab3Init;
        public PathChain Grab3Grab;
        public PathChain Shoot4;
        public PathChain Grab4Init;
        public PathChain Grab4Grab;
        public PathChain Shoot5;
        public PathChain MoveOffLaunchLine;

        public Paths() {
            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 16.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(113.6))
                    .build();

            Grab1Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 16.000), new Pose(12.000, 20.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(113.63),
                            Math.toRadians(225)
                    )
                    .build();

            Grab1Grab = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(12.000, 20.000), new Pose(8.000, 8.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(270))
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(8.000, 8.000), new Pose(56.000, 16.000)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(270),
                            Math.toRadians(113.63)
                    )
                    .build();

            Grab2Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 16.000), new Pose(50.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113.6), Math.toRadians(180))
                    .build();

            Grab2Grab = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 36.000), new Pose(18.000, 36.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 36.000), new Pose(56.000, 16.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(113.6))
                    .build();

            Grab3Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 16.000), new Pose(50.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(113.6), Math.toRadians(180))
                    .build();

            Grab3Grab = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 60.000), new Pose(18.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 60.000), new Pose(60.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Grab4Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.000, 84.000), new Pose(60.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Grab4Grab = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.000, 84.000), new Pose(18.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 84.000), new Pose(60.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            MoveOffLaunchLine = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.000, 84.000), new Pose(30.000, 70.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
        }
    }


    private ElapsedTime timer = new ElapsedTime();

    private Pose goalPose = new Pose(4, 132);

    private MotorEx intake = new MotorEx("FlyWheelL");
    private Command intakeOn = new SetPower(intake, -1);
    private Command intakeOff = new SetPower(intake, 0);

    private Command shoot = new SequentialGroup(
            Shooter.INSTANCE.onFromDistSupplier(() -> PedroComponent.follower().getPose().distanceFrom(goalPose)),
            new WaitUntil(() -> Shooter.INSTANCE.on.isDone()),
            intakeOn,
            new Delay(1),
            new ParallelGroup(
                    Shooter.INSTANCE.setSpeedCommand(0),
                    intakeOff
            )
    );

    @Override
    public void onStartButtonPressed() {
        Drawing.init();

        PedroComponent.follower().setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        Paths paths = new Paths();

        new SequentialGroup(
                new FollowPath(paths.Shoot1, true, 0.5),
                shoot, //shoot
                new FollowPath(paths.Grab1Init),
                intakeOn,
                new FollowPath(paths.Grab1Grab, true, 0.5),
                intakeOff,
                new FollowPath(paths.Shoot2),
                shoot, //shoot
                new FollowPath(paths.Grab2Init),
                intakeOn,
                new FollowPath(paths.Grab2Grab, true, 0.5),
                intakeOff,
                new FollowPath(paths.Shoot3),
                shoot, //shoot
                new FollowPath(paths.Grab3Init),
                intakeOn,
                new FollowPath(paths.Grab3Grab, true, 0.5),
                intakeOff,
                new FollowPath(paths.Shoot4),
                shoot, //shoot
                new FollowPath(paths.Grab4Init),
                intakeOn,
                new FollowPath(paths.Grab4Grab, true, 0.5),
                intakeOff,
                new FollowPath(paths.Shoot5),
                shoot, //shoot
                new FollowPath(paths.MoveOffLaunchLine)
        ).schedule();

        timer.reset();
    }

    @Override
    public void onUpdate() {
        telemetryM.debug("time", timer.seconds());
        telemetryM.debug("position", PedroComponent.follower().getPose());
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());

        Drawing.drawDebug(PedroComponent.follower());
        telemetryM.update();
    }

    @Override
    public void onStop() {
        DataStorage.INSTANCE.onBlue = true;
        DataStorage.INSTANCE.teleopStartPose = PedroComponent.follower().getPose();
    }
}
