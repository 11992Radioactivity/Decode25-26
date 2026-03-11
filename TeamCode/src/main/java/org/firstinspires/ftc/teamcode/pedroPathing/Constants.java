package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
    // TODO: TUNE PEDRO (build with teamcode to avoid sloth problems with panels)
    // stolen tuning from 22131
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.1211,0.0017011))
            .forwardZeroPowerAcceleration(-35.023)
            .lateralZeroPowerAcceleration(-56.549)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0, .01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.015,0,0.003,0))
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true);

    // manually tuned
    /*public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.65)
            .forwardZeroPowerAcceleration(-39.840248)
            .lateralZeroPowerAcceleration(-52.597607)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.01,
                    0.03
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.5,
                    0,
                    0.1,
                    0.03
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.012,
                    0,
                    0,
                    0.6,
                    0.05
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.003,
                    0.6,
                    0.05
            ))
            .drivePIDFSwitch(15)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0005);
    */

    /*
    .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.025,
                    0,
                    0.0005,
                    0.015
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0,
                    0.00125,
                    0.1
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.025,
                    0,
                    0.0005,
                    0.6,
                    0.01
            ))
     */

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
            .xVelocity(70.1223)
            .yVelocity(47.1268);

    // TODO: FIND NEW POSITIONS AFTER MOVING DRIVE MOTORS BACK
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.32)//-0.964)//0.879)//1.32)
            .strafePodX(-3.56)//2.217)//-2.696)//-3.56)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}