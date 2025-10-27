package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.*;
import org.firstinspires.ftc.teamcode.subsystems.DoubleShooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Autonomous(name = "Pedro Auto", preselectTeleOp = "NextFTCTeleop")
public class PedroAuto extends NextFTCOpMode {
    public PedroAuto() {
        addComponents(
                //new SubsystemComponent(DoubleShooter.INSTANCE),
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

        public Follower follower = PedroComponent.follower();
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths() {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.300, 122.600), new Pose(36.000, 107.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(135))
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
                            new BezierLine(new Pose(50.000, 84.500), new Pose(16.000, 84.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.000, 84.500), new Pose(36.000, 107.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }

    @Override
    public void onStartButtonPressed() {
        Drawing.init();

        PedroComponent.follower().setStartingPose(new Pose(20.300, 122.600, Math.toRadians(325)));

        Paths paths = new Paths();

        new SequentialGroup(
                new FollowPath(paths.Path1, true, 0.5),
                new Delay(3), //shoot
                new FollowPath(paths.Path2, true, 0.5),
                new FollowPath(paths.Path3, true, 0.5),
                new FollowPath(paths.Path4, true, 0.5),
                new Delay(3) //shoot
        ).schedule();
    }

    @Override
    public void onUpdate() {
        telemetryM.debug("position", PedroComponent.follower().getPose());
        telemetryM.debug("velocity", PedroComponent.follower().getVelocity());

        Drawing.drawDebug(PedroComponent.follower());
        telemetryM.update();
    }
}
