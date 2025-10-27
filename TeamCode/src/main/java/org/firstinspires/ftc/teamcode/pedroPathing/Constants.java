package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.8)
            .forwardZeroPowerAcceleration(-39.840248)
            .lateralZeroPowerAcceleration(-52.597607)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03,
                    0,
                    0,
                    0.015
            ))
            //.translationalPIDFSwitch(4)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0,
                    0.01
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1,
                    0,
                    0.000005,
                    0.6,
                    0.01
            ))
            //.drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    // TODO: TUNE PEDRO
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_motor")
            .rightRearMotorName("back_right_motor")
            .leftRearMotorName("back_left_motor")
            .leftFrontMotorName("front_left_motor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(71.0092)
            .yVelocity(65.181987);

    // TODO: FIND NEW POSITIONS AFTER MOVING DRIVE MOTORS BACK
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-96)
            .strafePodX(48)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}