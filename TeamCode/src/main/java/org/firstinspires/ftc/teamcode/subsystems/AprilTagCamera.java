package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagCamera {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detections = new ArrayList<>();

    public AprilTagCamera(HardwareMap hw) {
        /*
        Focals (pixels) - Fx: 680.441 Fy: 680.441
        Optical center - Cx: 294.849 Cy: 176.897
        Radial distortion (Brown's Model)
        K1: -0.42101 K2: 0.169578 K3: 0.0583968
        P1: -0.000516989 P2: 0.00647042
        Skew: 0
         */

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(680.441, 680.441, 294.849, 176.897)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, 5.75, -3, 10.5, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 8, -80, 0, 0))
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();

        //PanelsCameraStream.INSTANCE.startStream(visionPortal, 15);
    }

    public void update() {
        detections = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetections() {
        return detections;
    }

    public AprilTagDetection getTagFromId(int id) {
        for (AprilTagDetection d : detections) {
            if (d.id == id) {
                return d;
            }
        }
        return null;
    }

    // convert ftc field coords to pedro coords
    public Pose getRobotPoseFromTag(AprilTagDetection detection) {
        Position position = detection.robotPose.getPosition().toUnit(DistanceUnit.INCH);
        double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
        return new Pose(position.y + 72, -position.x + 72, heading);
    }

    public double getAngleFromTag(AprilTagDetection detection) {
        AprilTagPoseFtc currentPose = detection.ftcPose;
        return currentPose.bearing;
    }

    public double getDistFromTag(AprilTagDetection detection) {
        AprilTagPoseFtc currentPose = detection.ftcPose;
        return currentPose.range;
    }

    public void displayTag(AprilTagDetection detection, JoinedTelemetry telemetry) {
        if (detection.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        }
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
            //PanelsCameraStream.INSTANCE.stopStream();
        }
    }
}
