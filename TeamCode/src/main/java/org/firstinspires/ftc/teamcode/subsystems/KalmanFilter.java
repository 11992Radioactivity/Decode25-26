package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class KalmanFilter {
    private double estimate; // current sensor fused estimate
    private double variance; // current distrust of the relative sensor
    private boolean angle = false; // if true, wraps around +-pi

    // higher process noise = lose trust in relative sensor quicker
    // higher measure noise = more smoothing of absolute sensor
    private final double process_noise;
    private final double measure_noise;

    public KalmanFilter(double estimate, double variance, double process_noise, double measure_noise, boolean angle_var) {
        this.estimate = estimate;
        this.variance = variance;
        this.process_noise = process_noise;
        this.measure_noise = measure_noise;
        angle = angle_var;
    }

    public KalmanFilter(double process_noise, double measure_noise, boolean angle_var) {
        this(0, 1e-6, process_noise, measure_noise, angle_var);
    }

    public KalmanFilter(double process_noise, double measure_noise) {
        this(process_noise, measure_noise, false);
    }

    public void updateProcess(double update) {
        estimate = update;
        variance += process_noise;
    }

    // low variance = set to process estimate
    // high variance = set to measurement
    public void updateMeasurement(double measurement) {
        double k = variance / (variance + measure_noise);
        if (!angle) {
            estimate += k * (measurement - estimate);
        } else {
            double error = AngleUnit.normalizeRadians(measurement - estimate);
            estimate += AngleUnit.normalizeRadians(estimate + k * error);
        }
        variance *= (1 - k);
    }

    public double getEstimate() {
        return estimate;
    }
}
