package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.*;
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

@Autonomous(name = "Blue Close 12 Auto", preselectTeleOp = "ManualTeleOp")
public class BlueClose12Auto extends NextFTCOpMode {
    public BlueClose12Auto() {
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

        public Pose shootPose = new Pose(40.000, 100.000);

        public Paths() {
            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.300, 122.600), shootPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(135))
                    .build();

            Grab1Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPose, new Pose(50.000, 82.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Grab1Grab = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 82.000), new Pose(18.000, 82.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            GatePush = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.000, 82.000),
                                    new Pose(35.229, 71.119),
                                    new Pose(14.000, 70.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 84.500), shootPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Grab2Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPose, new Pose(50.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Grab2Grab = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 60.000), new Pose(10.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.000, 60.000),
                                    new Pose(41.394, 73.321),
                                    shootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Grab3Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPose, new Pose(50.000, 37.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Grab3Grab = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 37.000), new Pose(10.000, 37.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 36.000), shootPose)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            MoveOffLaunchLine = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPose, new Pose(30.000, 80.00))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
        }
    }

    private ElapsedTime timer = new ElapsedTime();

    private Pose goalPose = new Pose(4, 132);

    private MotorEx intake = new MotorEx("Intake");
    private MotorEx transfer = new MotorEx("Transfer");
    private Command intakeOn = new SetPower(intake, 1);
    private Command intakeOff = new SetPower(intake, 0);
    private Command transferOn = new SetPower(transfer, -1);
    private Command transferOff = new SetPower(transfer, 0);

    @Override
    public void onInit() {
        Shooter.INSTANCE.setSpeed(0);;
    }

    @Override
    public void onStartButtonPressed() {
        Drawing.init();

        PedroComponent.follower().setStartingPose(new Pose(20.300, 122.600, Math.toRadians(140)));

        Command shoot = new SequentialGroup(
                Shooter.INSTANCE.openGate,
                new Delay(0.7),
                intakeOn,
                transferOn,
                new Delay(1.5),
                intakeOff,
                transferOff,
                new ParallelGroup(Shooter.INSTANCE.closeGate, intakeOff)
        );

        Paths paths = new Paths();

        new SequentialGroup(
                Shooter.INSTANCE.setSpeedCommand(2250),
                new FollowPath(paths.Shoot1),
                shoot, //shoot
                new FollowPath(paths.Grab1Init),
                intakeOn,
                new FollowPath(paths.Grab1Grab, true, 0.7),
                new FollowPath(paths.GatePush, true, 0.5),
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
        Shooter.INSTANCE.off.schedule();
        DataStorage.INSTANCE.onBlue = true;
        DataStorage.INSTANCE.teleopStartPose = PedroComponent.follower().getPose();
    }
}
