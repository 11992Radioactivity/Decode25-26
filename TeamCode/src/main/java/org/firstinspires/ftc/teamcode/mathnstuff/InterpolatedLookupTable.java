package org.firstinspires.ftc.teamcode.mathnstuff;

import java.util.NavigableMap;
import java.util.TreeMap;

public class InterpolatedLookupTable {

    private final NavigableMap<Double, Double> table = new TreeMap<>();

    public InterpolatedLookupTable() {}

    public InterpolatedLookupTable(double... params) {
        for (int i = 0; i < params.length; i += 2) {
            double distance = params[i];
            double velocity = params[i + 1];
            table.put(distance, velocity);
        }
    }

    /** Add a distance -> velocity entry */
    public void addPoint(double distance, double velocity) {
        table.put(distance, velocity);
    }

    /** Get interpolated velocity for a given distance */
    public double get(double distance) {
        if (table.isEmpty()) {
            throw new IllegalStateException("Lookup table is empty");
        }

        // Exact match
        if (table.containsKey(distance)) {
            return table.get(distance);
        }

        // Nearest lower and upper entries
        var lower = table.floorEntry(distance);
        var upper = table.ceilingEntry(distance);

        // Clamp if outside range
        if (lower == null) {
            return upper.getValue();
        }
        if (upper == null) {
            return lower.getValue();
        }

        // Linear interpolation
        double x1 = lower.getKey();
        double y1 = lower.getValue();
        double x2 = upper.getKey();
        double y2 = upper.getValue();

        double t = (distance - x1) / (x2 - x1);
        return y1 + t * (y2 - y1);
    }
}
