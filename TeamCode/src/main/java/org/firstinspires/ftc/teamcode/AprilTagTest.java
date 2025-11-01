package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagCamera;
import org.firstinspires.ftc.teamcode.subsystems.DoubleShooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class AprilTagTest extends NextFTCTeleop {
    public AprilTagTest() {
        addComponents(
                BulkReadComponent.INSTANCE
        );
    }

    private AprilTagCamera camera;
    private JoinedTelemetry joinedTelemetry;

    @Override
    public void onStartButtonPressed() {
        joinedTelemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        camera = new AprilTagCamera(hardwareMap, joinedTelemetry);
    }

    @Override
    public void onUpdate() {
        camera.update();
        AprilTagDetection blueGoal = camera.getTagFromId(20);
        //blueGoal.ftcPose.
        if (blueGoal != null)
            camera.displayTag(blueGoal);
        else
            joinedTelemetry.addLine("no tag found");
        joinedTelemetry.update();
    }
}
