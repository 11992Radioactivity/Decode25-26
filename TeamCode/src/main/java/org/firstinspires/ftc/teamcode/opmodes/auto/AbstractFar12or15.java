package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mathnstuff.DataStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSensors;
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
public abstract class AbstractFar12or15 extends NextFTCOpMode {
    final boolean blue;
    final boolean hptwice; // true if good partner close auto to grab a bunch from hp
    final Pose goal;
    JoinedTelemetry telemetryM;
    ElapsedTime timer;
    double last_time = 0;
    double cur_time = 0;
    boolean done = false;
    boolean intake_on = false;
    boolean shooting = false;
    IntakeSensors intakeSensors;
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
            new Delay(0.3),
            new Command() {
                @Override
                public boolean isDone() {
                    intake_on = true;
                    shooting = true;
                    return true;
                }
            },
            intakeOn,
            transferOn,
            new Delay(0.7),
            new Command() {
                @Override
                public boolean isDone() {
                    intake_on = false;
                    shooting = false;
                    return true;
                }
            },
            intakeOff,
            transferOff,
            Shooter.INSTANCE.closeGate
    );

    class Paths {
        PathChain PreloadShoot;
        PathChain GrabSpike1;
        PathChain Shoot1;
        PathChain InitSpike2;
        PathChain GrabSpike2;
        PathChain Shoot2;
        PathChain HumanPlayerGrab1;
        PathChain HumanPlayerGrab2;
        PathChain Shoot3;
        PathChain Leave;

        Pose shoot = new Pose(56, 18, Math.toRadians(110));
        Pose spike1control = new Pose(53, 40);
        Pose spike1 = new Pose(8, 36, Math.toRadians(180));
        Pose spike2init = new Pose(12, 24, Math.toRadians(230));
        Pose spike2grab = new Pose(12, 12, Math.toRadians(270));
        Pose hpgrab1 = new Pose(11, 10, Math.toRadians(115));
        Pose hpgrab2 = new Pose(11, 36, Math.toRadians(115));
        Pose leave = new Pose(30, 24, Math.toRadians(90));

        public Paths(Pose start) {
            if (!blue) {
                shoot = shoot.mirror();
                spike1control = spike1control.mirror();
                spike1 = spike1.mirror();
                spike2init = spike2init.mirror();
                spike2grab = spike2grab.mirror();
                hpgrab1 = hpgrab1.mirror();
                hpgrab2 = hpgrab2.mirror();
                leave = leave.mirror();
            }

            Follower follower = PedroComponent.follower();
            PreloadShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    start,
                                    shoot
                            )
                    ).setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading(), 0.5)
                    //.setNoDeceleration()
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
                    //.setNoDeceleration()
                    .build();
            InitSpike2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shoot,
                                    spike2init
                            )
                    ).setLinearHeadingInterpolation(shoot.getHeading(), spike2init.getHeading(), 0.5)
                    //.setNoDeceleration()
                    .build();
            GrabSpike2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    spike2init,
                                    spike2grab
                            )
                    ).setLinearHeadingInterpolation(spike2init.getHeading(), spike2grab.getHeading(), 0.5)
                    //.setNoDeceleration()
                    .build();
            Shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    spike2grab,
                                    shoot
                            )
                    ).setLinearHeadingInterpolation(spike2grab.getHeading(), shoot.getHeading(), 0.5)
                    //.setNoDeceleration()
                    .build();
            HumanPlayerGrab1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shoot,
                                    hpgrab1
                            )
                    ).setLinearHeadingInterpolation(shoot.getHeading(), hpgrab1.getHeading(), 0.5)
                    .setBrakingStrength(2)
                    .build();
            HumanPlayerGrab2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    hpgrab1,
                                    hpgrab2
                            )
                    ).setLinearHeadingInterpolation(hpgrab1.getHeading(), hpgrab2.getHeading(), 0.5)
                    //.setNoDeceleration()
                    .build();
            Shoot3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    hpgrab2,
                                    shoot
                            )
                    ).setLinearHeadingInterpolation(hpgrab2.getHeading(), shoot.getHeading(), 0.5)
                    .setBrakingStrength(2)
                    .build();
            Leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shoot,
                                    leave
                            )
                    ).setLinearHeadingInterpolation(shoot.getHeading(), leave.getHeading(), 0.5)
                    //.setNoDeceleration()
                    .build();
        }
    }

    public AbstractFar12or15(boolean on_blue, boolean intake_hp_twice) {
        blue = on_blue;
        hptwice = intake_hp_twice;

        if (blue) {
            goal = new Pose(4, 140);
        } else {
            goal = new Pose(140, 140);
        }

        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    @Override
    public void onInit() {
        Shooter.INSTANCE.setSpeed(0);

        intakeSensors = new IntakeSensors(hardwareMap);
    }

    @Override
    public void onStartButtonPressed() {
        telemetryM = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        timer = new ElapsedTime();

        Drawing.init();

        Pose start = new Pose(56, 8, Math.toRadians(90));
        if (!blue) start = start.mirror();
        PedroComponent.follower().setStartingPose(start);

        Paths paths = new Paths(start);

        Command thirdGrab;
        if (hptwice) {
            thirdGrab = new SequentialGroup(
                    intakeOn,
                    new FollowPath(paths.HumanPlayerGrab1),
                    new Delay(0.25),
                    new FollowPath(paths.HumanPlayerGrab2),
                    new Delay(0.25),
                    new FollowPath(paths.Shoot3),
                    intakeOff,
                    shootCommand
            );
        } else {
            // do nothing
            thirdGrab = new Command() {
                @Override
                public boolean isDone() {
                    return true;
                }
            };
        }

        (new SequentialGroup(
                Shooter.INSTANCE.setSpeedCommand(2975),
                new FollowPath(paths.PreloadShoot),
                new Delay(1),
                shootCommand,
                intakeOn,
                new FollowPath(paths.GrabSpike1),
                new Delay(0.1),
                new FollowPath(paths.Shoot1),
                intakeOff,
                shootCommand,
                intakeOn,
                new FollowPath(paths.InitSpike2),
                new FollowPath(paths.GrabSpike2, true, 0.55),
                new Delay(0.35),
                new FollowPath(paths.Shoot2),
                intakeOff,
                shootCommand,
                thirdGrab,
                intakeOn,
                new FollowPath(paths.HumanPlayerGrab1),
                new Delay(0.25),
                new FollowPath(paths.HumanPlayerGrab2),
                new Delay(0.25),
                new FollowPath(paths.Shoot3),
                intakeOff,
                shootCommand,
                Shooter.INSTANCE.setSpeedCommand(0),
                new FollowPath(paths.Leave),
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
        if (!opModeInInit()) {
            Shooter.INSTANCE.setHoodPos(0.455);
        }

        double dt = timer.seconds() - last_time;
        last_time = timer.seconds();

        if (!done) {
            DataStorage.INSTANCE.onBlue = blue;
            DataStorage.INSTANCE.teleopStartPose = PedroComponent.follower().getPose();
            cur_time = timer.time();
        } else {
            Shooter.INSTANCE.setSpeed(0);
        }

        intakeSensors.updateCounter(3);

        if (intake_on && intakeSensors.counterGreaterThan(0.7) && !shooting) {
            intake.setPower(0);
        } else if (intake_on) {
            intake.setPower(1);
        }

        telemetryM.addData("hz", 1 / dt);
        telemetryM.addData("time", cur_time);
        telemetryM.addData("hood pos", Shooter.INSTANCE.hood_pos);
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
