package com.example.movedistance;

import java.util.List;
import java.util.Map;

public class IMUDataParser {
    public static Map<String, List<String>> parseSensorGroups(String[] csvHeader) {

        return java.util.Collections.emptyMap();
    }

    public static double[][] cutIMU(String sensor, List<String> sensorColumns, List<Map<String, Object>> imuData) {
        return new double[0][];
    }

    public static double[][][] reshapeIMU(double[][] cutIMU, int i, int size) {
        return new double[0][][];
    }

    public static Map<String, Object> getTimestampUnique(List<Map<String, Object>> imuData) {
        return null;
    }
}
