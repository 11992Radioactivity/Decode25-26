package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;

// store data between auto and teleop
public class DataStorage {
    public static final DataStorage INSTANCE = new DataStorage();
    private DataStorage() {}

    public boolean onBlue = true;
    public Pose teleopStartPose = new Pose(72, 72);
}
