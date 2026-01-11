package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Blue Far Auto", preselectTeleOp = "ManualTeleOp")
public class BlueFar extends NextFTCOpMode {
    public BlueFar() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public class Paths {

        public PathChain Shoot1;
        public PathChain Grab1Init;
        public PathChain Grab1Grab;
        public PathChain Grab1Init2;
        public PathChain Grab1Grab2;
        public PathChain Shoot2;
        public PathChain GoPark;
        public Follower follower = PedroComponent.follower();

        public Paths() {
            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(59.000, 16.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();
            Grab1Init = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.000, 16.000), new Pose(20.000, 9.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(180))
                    .build();

            Grab1Grab = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 9.000), new Pose(14.000, 9.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Grab1Init2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(10.000, 9.000), new Pose(20.000, 9.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Grab1Grab2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(20.000, 9.000), new Pose(10.000, 9.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Shoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 9.000), new Pose(59.000, 16.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112.5))
                    .build();

            GoPark = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.000, 16.000), new Pose(16.000, 16.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    private ElapsedTime timer = new ElapsedTime();

    private Pose goalPose = new Pose(4, 132);

    private MotorEx intake = new MotorEx("Intake");
    private MotorEx transfer = new MotorEx("Transfer");
    private Command intakeOn = new SetPower(intake, -1);
    private Command intakeOff = new SetPower(intake, 0);
    private Command transferOnHalf = new SetPower(transfer, -0.5);
    private Command transferOn = new SetPower(transfer, -1);
    private Command transferOff = new SetPower(transfer, 0);

    private Command shootIndividual = new SequentialGroup(
            intakeOn,
            transferOnHalf,
            new Delay(0.1),
            intakeOff,
            transferOff,
            new Delay(1.0)
    );

    private Command shoot = new SequentialGroup(
            Shooter.INSTANCE.onFromDistSupplier(() -> PedroComponent.follower().getPose().distanceFrom(goalPose) - 6),
            Shooter.INSTANCE.openGate,
            new Delay(1.0),
            shootIndividual,
            shootIndividual,
            intakeOn,
            transferOn,
            new Delay(0.5),
            intakeOff,
            transferOff,
            new ParallelGroup(
                    Shooter.INSTANCE.setSpeedCommand(3400),
                    new ParallelGroup(Shooter.INSTANCE.closeGate, intakeOff)
            )
    );

    @Override
    public void onStartButtonPressed() {
        Drawing.init();

        PedroComponent.follower().setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        Paths paths = new Paths();

        new SequentialGroup(
                Shooter.INSTANCE.setSpeedCommand(3400),
                new FollowPath(paths.Shoot1, true, 0.5),
                shoot, //shoot
                new FollowPath(paths.Grab1Init),
                intakeOn,
                new FollowPath(paths.Grab1Grab, true, 0.7),
                new FollowPath(paths.Grab1Init2, true, 0.7),
                new FollowPath(paths.Grab1Grab2, true, 0.7),
                new FollowPath(paths.Shoot2, true, 0.7),
                intakeOff,
                shoot,
                new FollowPath(paths.GoPark)
                //Shooter.INSTANCE.off//shoot
                /*new FollowPath(paths.Grab3Init),
                intakeOn,
                new FollowPath(paths.Grab3Grab, true, 0.7),
                new FollowPath(paths.Shoot4),
                intakeOff,
                shoot, //shoot
                new FollowPath(paths.MoveOffLaunchLine)*/
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
