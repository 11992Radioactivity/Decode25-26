package org.firstinspires.ftc.teamcode.mathnstuff;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class KalmanFilter {
    private double estimate; // current sensor fused estimate
    private double variance; // current distrust of the relative sensor
    private boolean angle = false; // if true, wraps around +-pi

    // higher process noise = lose trust in relative sensor quicker
    // higher measure noise = more smoothing of absolute sensor
    private double process_noise;
    private double measure_noise;
    private double rejection_amount = Double.POSITIVE_INFINITY;
    private double override_amount = 10; // estimate so bad that even a measurement with a lot of variance will be more accurate

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

    public void setRejectionAmount(double amt) {
        rejection_amount = amt;
    }

    public void setOverrideAmount(double amt) {
        override_amount = amt;
    }

    // this is a weird implementation but it works with my code so ignore it
    // normally it works by adding the update to the current state but since pedro
    // does stuff automatically it's easier to do this
    public void updateProcess(double update) {
        estimate = update;
        variance += process_noise;
    }

    // low variance = set to process estimate
    // high variance = set to measurement
    public void updateMeasurement(double measurement, double measure_noise_scale) {
        double err = Math.abs(measurement - estimate);
        if (err > rejection_amount && err < override_amount) return;

        double k = variance / (variance + measure_noise * measure_noise_scale);
        if (!angle) {
            estimate += k * (measurement - estimate);
        } else {
            double error = AngleUnit.normalizeRadians(measurement - estimate);
            estimate += AngleUnit.normalizeRadians(estimate + k * error);
        }
        variance *= (1 - k);
    }

    public void updateMeasurement(double measurement) {
        updateMeasurement(measurement, 1);
    }

    public double getEstimate() {
        return estimate;
    }
}
