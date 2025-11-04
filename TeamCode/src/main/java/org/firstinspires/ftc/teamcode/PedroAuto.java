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

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Autonomous(name = "Pedro Auto", preselectTeleOp = "NextFTCTeleop")
public class PedroAuto extends NextFTCOpMode {
    public PedroAuto() {
        addComponents(
                //new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left_motor").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right_motor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left_motor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right_motor").brakeMode();
    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public static class Paths {

        private Follower follower = PedroComponent.follower();
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;

        public Paths() {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.300, 122.600), new Pose(36.000, 107.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.000, 107.500), new Pose(50.000, 84.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 84.500), new Pose(18.000, 84.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 84.500), new Pose(36.000, 107.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.000, 107.500), new Pose(50.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 60.000), new Pose(18.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(18.000, 60.000),
                                    new Pose(41.394, 73.321),
                                    new Pose(36.000, 107.500)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.000, 107.500), new Pose(50.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 36.000), new Pose(18.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.000, 36.000), new Pose(36.000, 107.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void onStartButtonPressed() {
        Drawing.init();

        PedroComponent.follower().setStartingPose(new Pose(20.300, 122.600, Math.toRadians(145)));

        Paths paths = new Paths();

        new SequentialGroup(
                new FollowPath(paths.Path1, true, 0.5),
                new Delay(2), //shoot
                new FollowPath(paths.Path2),
                new FollowPath(paths.Path3, true, 0.5),
                new FollowPath(paths.Path4),
                new Delay(2), //shoot
                new FollowPath(paths.Path5),
                new FollowPath(paths.Path6, true, 0.5),
                new FollowPath(paths.Path7),
                new Delay(2), //shoot
                new FollowPath(paths.Path8),
                new FollowPath(paths.Path9, true, 0.5),
                new FollowPath(paths.Path10),
                new Delay(2) //shoot
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
}
