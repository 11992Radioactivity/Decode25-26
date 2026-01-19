package org.firstinspires.ftc.teamcode.mathnstuff;

import com.pedropathing.control.LowPassFilter;
import com.pedropathing.geometry.Pose;

public class PoseEstimator {
    private Pose currentEstimate;
    private final LowPassFilter xLowPass, yLowPass, hLowPass; // smooth out vision data
    private final KalmanFilter xKalman, yKalman, hKalman; // combine odom and vision data

    // variance = standard deviation squared
    //  - so if vision is +- 5 inches off, put in 25 for linear
    //  - this works because it uses ratios between the sensors' variance
    // smoothing is between 0-1 and is the amount of change accepted per update
    //  - so if it's 1 it doesn't smooth at all, but 0.5 it mixes half previous with half new
    public PoseEstimator(Pose initialPose, double odomLinearVariance, double odomHeadingVariance, double visionLinearVariance, double visionHeadingVariance, double visionSmoothing) {
        currentEstimate = initialPose;
        xLowPass = new LowPassFilter(visionSmoothing);
        yLowPass = new LowPassFilter(visionSmoothing);
        hLowPass = new LowPassFilter(visionSmoothing);
        xKalman = new KalmanFilter(odomLinearVariance, visionLinearVariance);
        yKalman = new KalmanFilter(odomLinearVariance, visionLinearVariance);
        hKalman = new KalmanFilter(odomHeadingVariance, visionHeadingVariance);

        xKalman.setRejectionAmount(3);
        yKalman.setRejectionAmount(3);
        hKalman.setRejectionAmount(Math.toRadians(5));
    }

    public PoseEstimator(Pose initialPose) {
        this(initialPose,
                Math.pow(0.25, 2),
                Math.pow(Math.toRadians(3), 2),
                Math.pow(5, 2),
                Math.pow(Math.toRadians(20), 2),
                0.6);
    }

    public Pose getCurrentEstimate() {
        return currentEstimate;
    }

    // on a loop with only odometry, only call this
    public void updateOdometry(Pose odomPose) {
        xKalman.updateProcess(odomPose.getX());
        yKalman.updateProcess(odomPose.getY());
        hKalman.updateProcess(odomPose.getHeading());

        currentEstimate = new Pose(
                xKalman.getEstimate(),
                yKalman.getEstimate(),
                hKalman.getEstimate()
        );
    }

    // on a loop with odometry and vision, call updateOdometry and then this
    // pass distance to tag in order to scale the variance for less trust farther away
    public void updateVision(Pose visionPose, double distance) {
        xLowPass.update(visionPose.getX(), 0);
        yLowPass.update(visionPose.getY(), 0);
        hLowPass.update(visionPose.getHeading(), 0);

        // scale the variance by 1 extra every 32 inches further from april tag
        double scale = Math.max(1, distance / 32);
        xKalman.updateMeasurement(xLowPass.getState(), scale);
        yKalman.updateMeasurement(yLowPass.getState(), scale);
        hKalman.updateMeasurement(hLowPass.getState());

        currentEstimate = new Pose(
                xKalman.getEstimate(),
                yKalman.getEstimate(),
                hKalman.getEstimate()
        );
    }
}
