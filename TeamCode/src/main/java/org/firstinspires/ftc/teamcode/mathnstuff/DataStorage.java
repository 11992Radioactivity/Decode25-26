package org.firstinspires.ftc.teamcode.mathnstuff;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;

// store data between auto and teleop
public class DataStorage {
    private static final String FILE_NAME = "match_data.txt";
    private static final long MAX_AGE_MS = 60_000; // 30 seconds

    private static File getFile() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public static void save(Pose pose, boolean blue) {
        long now = System.currentTimeMillis();

        String data = pose.getX() + "," + pose.getY() + "," + pose.getHeading() + "," + now + "," + blue;

        // chatgpt said to make the write atomic which sounds cool
        File file = getFile();
        File tempFile = new File(file.getAbsolutePath() + ".tmp");
        ReadWriteFile.writeFile(tempFile, data);
        if (!tempFile.renameTo(file)) {
            ReadWriteFile.writeFile(file, data);
        }
    }

    private static String[] loadData() {
        File file = getFile();

        if (!file.exists()) return null;

        try {
            String content = ReadWriteFile.readFile(file);
            String[] parts = content.split(",");

            if (parts.length != 5) return null;

            return parts;
        } catch (Exception e) {
            return null;
        }
    }

    public static Pose loadPose() {
        String[] parts = loadData();

        if (parts == null) return new Pose();

        try {
            double x = Double.parseDouble(parts[0]);
            double y = Double.parseDouble(parts[1]);
            double heading = Double.parseDouble(parts[2]);
            long timestamp = Long.parseLong(parts[3]);

            if (System.currentTimeMillis() - timestamp < MAX_AGE_MS)
                return new Pose(x, y, heading);
            else
                return new Pose(72, 72);
        } catch (Exception e) {
            return new Pose(72, 72);
        }
    }

    public static boolean loadAlliance() {
        String[] parts = loadData();

        if (parts == null) return true;

        try {
            long timestamp = Long.parseLong(parts[3]);
            boolean blue = Boolean.parseBoolean(parts[4]);

            if (System.currentTimeMillis() - timestamp < MAX_AGE_MS)
                return blue;
            else
                return true;
        } catch (Exception e) {
            return true;
        }
    }
}