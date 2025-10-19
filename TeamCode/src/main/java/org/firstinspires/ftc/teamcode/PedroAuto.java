package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
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
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left_motor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("front_right_motor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left_motor").brakeMode().reversed();
    private final MotorEx backRightMotor = new MotorEx("back_right_motor").brakeMode();

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
                    .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(135))
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
        Paths paths = new Paths();
        Command p1 = new FollowPath(paths.Path1);
        Command p2 = new FollowPath(paths.Path2);
        Command p3 = new FollowPath(paths.Path3);
        Command p4 = new FollowPath(paths.Path4);

        SequentialGroup auto = new SequentialGroup(
                p1,
                new Delay(3000),//shoot
                p2,
                p3,
                p4,
                new Delay(3000)//shoot
        );

        auto.schedule();
    }
}
