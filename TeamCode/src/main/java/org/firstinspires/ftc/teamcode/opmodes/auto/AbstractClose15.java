package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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

/*
    One class, 4 autos :)))
 */
public abstract class AbstractClose15 extends NextFTCOpMode {
    final boolean blue;
    final boolean gatetwice; // true if good partner far auto to leave 3rd spike
    JoinedTelemetry telemetryM;
    ElapsedTime timer;
    double cur_time = 0;
    boolean done = false;
    boolean intake_on = false;
    final MotorEx intake = new MotorEx("Intake");
    final MotorEx transfer = new MotorEx("Transfer");
    final Command intakeOn = new Command() {
        @Override
        public boolean isDone() {
            intake_on = true;
            intake.setPower(1);
            return true;
        }
    };
    final Command intakeOff = new Command() {
        @Override
        public boolean isDone() {
            intake_on = false;
            intake.setPower(0);
            return true;
        }
    };
    final Command transferOn = new SetPower(transfer, -1);
    final Command transferOff = new SetPower(transfer, 0);
    final Command shootCommand = new SequentialGroup(
            Shooter.INSTANCE.openGate,
            new Delay(0.7),
            intakeOn,
            transferOn,
            new Delay(0.5),
            intakeOff,
            transferOff,
            Shooter.INSTANCE.closeGate
    );

    class Paths {
        PathChain PreloadShoot;
        PathChain GrabSpike1;
        PathChain Shoot1;
        PathChain OpenGate1;
        PathChain GrabGate1;
        PathChain Shoot2;
        PathChain GrabSpike2;
        PathChain Shoot3;
        PathChain GrabSpike3;
        PathChain Shoot4;

        Pose shoot = new Pose(48, 96, Math.toRadians(135));
        Pose spike1control = new Pose(65, 55);
        Pose spike1 = new Pose(10, 60, Math.toRadians(180));
        Pose gatecontrol = new Pose(65, 55);
        Pose gate = new Pose(12, 61, Math.toRadians(145));
        Pose grabgate = new Pose(11, 56, Math.toRadians(145));
        Pose spike2control = new Pose(65, 23);
        Pose spike2 = new Pose(10, 36, Math.toRadians(180));
        Pose shoot3control = new Pose(50, 50);
        Pose spike3control = new Pose(48.550, 80.725);
        Pose spike3 = new Pose(16, 82, Math.toRadians(180));
        Pose shoot4 = new Pose(60, 110, Math.toRadians(150));

        public Paths(Pose start) {
            if (!blue) {
                shoot = shoot.mirror();
                spike1control = spike1control.mirror();
                spike1 = spike1.mirror();
                gatecontrol = gatecontrol.mirror();
                gate = gate.mirror();
                grabgate = grabgate.mirror();
                spike2control = spike2control.mirror();
                spike2 = spike2.mirror();
                shoot3control = shoot3control.mirror();
                spike3control = spike3control.mirror();
                spike3 = spike3.mirror();
                shoot4 = shoot4.mirror();
            }

            Follower follower = PedroComponent.follower();
            PreloadShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    start,
                                    shoot
                            )
                    ).setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading(), 0.5)
                    .setNoDeceleration()
                    .build();
            GrabSpike1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shoot,
                                    spike1control,
                                    spike1
                            )
                    ).setLinearHeadingInterpolation(shoot.getHeading(), spike1.getHeading(), 0.3)
                    .setBrakingStrength(2)
                    .build();
            Shoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    spike1,
                                    spike1control,
                                    shoot
                            )
                    ).setLinearHeadingInterpolation(spike1.getHeading(), shoot.getHeading(), 0.5)
                    .setNoDeceleration()
                    .build();
            OpenGate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shoot,
                                    gatecontrol,
                                    gate
                            )
                    ).setLinearHeadingInterpolation(shoot.getHeading(), gate.getHeading(), 0.5)
                    .setNoDeceleration()
                    .build();
            GrabGate1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    gate,
                                    grabgate
                            )
                    ).setConstantHeadingInterpolation(grabgate.getHeading())
                    .setNoDeceleration()
                    .build();
            Shoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    grabgate,
                                    gatecontrol,
                                    shoot
                            )
                    ).setLinearHeadingInterpolation(grabgate.getHeading(), shoot.getHeading(), 0.5)
                    .setNoDeceleration()
                    .build();
            GrabSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shoot,
                                    spike2control,
                                    spike2
                            )
                    ).setLinearHeadingInterpolation(shoot.getHeading(), spike2.getHeading(), 0.3)
                    .setBrakingStrength(2)
                    .build();
            Shoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    spike2,
                                    shoot3control,
                                    shoot
                            )
                    ).setLinearHeadingInterpolation(spike2.getHeading(), shoot.getHeading(), 0.5)
                    .setNoDeceleration()
                    .build();
            GrabSpike3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shoot,
                                    spike3control,
                                    spike3
                            )
                    ).setLinearHeadingInterpolation(shoot.getHeading(), spike3.getHeading(), 0.2)
                    .setBrakingStrength(2)
                    .build();
            Shoot4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    spike3,
                                    shoot4
                            )
                    ).setLinearHeadingInterpolation(spike3.getHeading(), shoot4.getHeading(), 0.5)
                    .setNoDeceleration()
                    .build();
        }
    }

    public AbstractClose15(boolean on_blue, boolean intake_gate_twice) {
        blue = on_blue;
        gatetwice = intake_gate_twice;

        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        Shooter.INSTANCE.setSpeed(0);
    }

    @Override
    public void onStartButtonPressed() {
        telemetryM = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        timer = new ElapsedTime();

        Drawing.init();

        Pose start = new Pose(20.300, 122.600, Math.toRadians(140));
        if (!blue) start = start.mirror();
        PedroComponent.follower().setStartingPose(start);

        Paths paths = new Paths(start);

        Command fourthGrab;
        if (gatetwice) {
            fourthGrab = new SequentialGroup(
                    intakeOn,
                    new FollowPath(paths.OpenGate1),
                    new Delay(0.25),
                    new FollowPath(paths.GrabGate1, true, 0.4),
                    new Delay(0.25),
                    new FollowPath(paths.Shoot2),
                    intakeOff
            );
        } else {
            fourthGrab = new SequentialGroup(
                    intakeOn,
                    new FollowPath(paths.GrabSpike2),
                    new Delay(0.25),
                    new FollowPath(paths.Shoot3),
                    intakeOff
            );
        }

        (new SequentialGroup(
                Shooter.INSTANCE.setSpeedCommand(2650),
                new FollowPath(paths.PreloadShoot),
                new Delay(0.5),
                shootCommand,
                intakeOn,
                new FollowPath(paths.GrabSpike1),
                new FollowPath(paths.Shoot1),
                intakeOff,
                shootCommand,
                intakeOn,
                new FollowPath(paths.OpenGate1),
                new Delay(0.25),
                new FollowPath(paths.GrabGate1, true, 0.4),
                new Delay(0.25),
                new FollowPath(paths.Shoot2),
                intakeOff,
                shootCommand,
                fourthGrab,
                shootCommand,
                intakeOn,
                new FollowPath(paths.GrabSpike3),
                new FollowPath(paths.Shoot4),
                intakeOff,
                shootCommand,
                Shooter.INSTANCE.setSpeedCommand(0),
                new Command() {
                    @Override
                    public boolean isDone() {
                        done = true;
                        return true;
                    }
                }
        )).schedule();
    }

    @Override
    public void onUpdate() {
        if (!done) {
            DataStorage.INSTANCE.onBlue = blue;
            DataStorage.INSTANCE.teleopStartPose = PedroComponent.follower().getPose();
            cur_time = timer.time();
        } else {
            Shooter.INSTANCE.setSpeed(0);
        }

        double intake_current = intake.getMotor().getCurrent(CurrentUnit.AMPS);
        if (intake_on && intake_current > 5 && intake.getPower() == 1) {
            intake.setPower(0);
        } else if (intake_on && intake_current < 5) {
            intake.setPower(1);
        }

        telemetryM.addData("time", cur_time);
        telemetryM.addData("position", PedroComponent.follower().getPose());
        telemetryM.addData("velocity", PedroComponent.follower().getVelocity());
        telemetryM.addData("shooter vel", Shooter.INSTANCE.getCurrentSpeed());
        telemetryM.addData("shooter target", Shooter.INSTANCE.targetSpeed);

        Drawing.drawDebug(PedroComponent.follower());
        telemetryM.update();
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.setSpeed(0);
    }
}
