package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;

import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
public class AprilTagTest extends NextFTCTeleop {
    public AprilTagTest() {
        addComponents(
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private AprilTagCamera camera;
    private JoinedTelemetry joinedTelemetry;
    private final MotorEx frontLeftMotor = new MotorEx("front_left_motor").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("front_right_motor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("back_left_motor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("back_right_motor").brakeMode();
    private AprilTagPoseFtc currentPose = new AprilTagPoseFtc(0, 18, 0, 0, 0, 0, 0, 0, 0);

    @Override
    public void onStartButtonPressed() {
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        camera = new AprilTagCamera(hardwareMap, joinedTelemetry);
    }

    @Override
    public void onUpdate() {
        camera.update();
        List<AprilTagDetection> tags = camera.getDetections();

        boolean lost;

        if (!tags.isEmpty()) {
            lost = false;
            AprilTagDetection tag = null;
            for (AprilTagDetection d : tags) {
                if (d.id != 20 && d.id != 24) {
                    tag = d;
                    break;
                }
            }
            if (tag == null) return;
            camera.displayTag(tag);
            currentPose = tag.ftcPose;
        } else {
            lost = true;
            currentPose = new AprilTagPoseFtc(0, 36, 0, 0, 0, 0, 0, 0, 0);
            joinedTelemetry.addLine("no tag found");
        }

        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                () -> -0.05 * (36 - currentPose.y),
                () -> -0.02 * (0 - currentPose.yaw),
                () -> {
                    if (!lost) return 0.02 * (0 - currentPose.bearing);
                    else return (double) 0;
                }
        );
        driverControlled.schedule();

        joinedTelemetry.update();
    }
}
