package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSensors {
    RevColorSensorV3 color1;
    RevColorSensorV3 color2;

    float h_min = 190.f;
    float s_min = 0.7f;
    int check_count = 0;
    int fount_count = 0;
    double ball_found = 0;
    double alpha = 0.8;

    public IntakeSensors(HardwareMap hw) {
        color1 = hw.get(RevColorSensorV3.class, "color1");
        color2 = hw.get(RevColorSensorV3.class, "color2");
    }

    private boolean colorInRange(float[] hsv) {
        // purple
        return hsv[0] > h_min || hsv[1] > s_min;
    }

    public boolean ballDetected() {
        float[] hsv1 = {0.f, 0.f, 0.f};
        float[] hsv2 = {0.f, 0.f, 0.f};

        Color.RGBToHSV(color1.red() * 255, color1.green() * 255, color1.blue() * 255, hsv1);
        Color.RGBToHSV(color2.red() * 255, color2.green() * 255, color2.blue() * 255, hsv2);

        return colorInRange(hsv1) || colorInRange(hsv2) || color1.getDistance(DistanceUnit.MM) < 20 || color2.getDistance(DistanceUnit.MM) < 20;
    }

    public double getCount() {
        return ball_found;
    }

    // avoid loop time interruptions
    public void updateCounter(double amt) {
        if (check_count > amt) {
            check_count = 0;
            if (ballDetected()) {
                ball_found = ball_found * alpha + 1 * (1 - alpha);
            } else {
                ball_found = ball_found * alpha + 0 * (1 - alpha);
            }
        } else {
            check_count++;
        }
    }

    public void resetCounter() {
        ball_found = 0;
    }

    public boolean counterGreaterThan(double amt) {
        return ball_found > amt;
    }

    public String get1Telem() {
        float[] hsv1 = {0.f, 0.f, 0.f};
        Color.RGBToHSV(color1.red() * 255, color1.green() * 255, color1.blue() * 255, hsv1);
        return hsv1[0] + " " + hsv1[1] + " " + hsv1[2];
    }

    public String get2Telem() {
        float[] hsv2 = {0.f, 0.f, 0.f};
        Color.RGBToHSV(color2.red() * 255, color2.green() * 255, color2.blue() * 255, hsv2);
        return hsv2[0] + " " + hsv2[1] + " " + hsv2[2];
    }
}
