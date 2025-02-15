package com.example.movedistance;

import java.util.*;
import java.util.stream.Collectors;

public class IMUDataParser {

    // ✅ 1. 센서 그룹을 파싱하는 메서드 (Python의 CONFIG["SENSORS"]와 유사)
    public static Map<String, List<String>> parseSensorGroups(String[] csvHeader) {
        Map<String, List<String>> sensorGroups = new HashMap<>();

        for (String column : csvHeader) {
            if (column.equals("timestamp") || column.equals("seq_num")) {
                continue; // 타임스탬프와 시퀀스 번호 제외
            }

            String[] parts = column.split("\\.");
            if (parts.length == 2) {
                String sensor = parts[0];  // 예: "gyro"
                sensorGroups.putIfAbsent(sensor, new ArrayList<>());
                if (!sensorGroups.get(sensor).contains(column)) {
                    sensorGroups.get(sensor).add(column);
                }
            } else {
                // 예외적인 경우 처리 (단일 컬럼, 예: "pressure.x")
                sensorGroups.putIfAbsent(column, new ArrayList<>());
                if (!sensorGroups.get(column).contains(column)) {
                    sensorGroups.get(column).add(column);
                }
            }
        }
        return sensorGroups;
    }

    // ✅ 2. 특정 센서의 데이터를 추출하는 메서드 (Python의 cut_imu()와 유사)
    public static double[][] cutIMU(String sensor, int numChannels, List<Map<String, Object>> imuData) {
        List<double[]> filteredData = new ArrayList<>();

        for (Map<String, Object> row : imuData) {
            double[] sensorValues = new double[numChannels];

            for (int i = 0; i < numChannels; i++) {
                String columnName = sensor + "." + (i == 0 ? "x" : i == 1 ? "y" : "z"); // ex: accel.x, accel.y, accel.z
                Object value = row.get(columnName);

                if (value instanceof Number) {
                    sensorValues[i] = ((Number) value).doubleValue();
                } else {
                    System.err.println("⚠ Warning: " + columnName + " 데이터가 누락되었습니다! 기본값 0을 사용합니다.");
                    sensorValues[i] = 0.0;
                }
            }
            filteredData.add(sensorValues);
        }

        if (filteredData.isEmpty()) {
            System.err.println("⚠ Warning: " + sensor + " 데이터가 없습니다! 기본값을 반환합니다.");
            return new double[100][numChannels]; // 기본값 반환
        }

        return filteredData.toArray(new double[filteredData.size()][]);
    }

    // ✅ 3. 데이터를 3D 배열로 변환하는 메서드 (Python의 reshape(-1, 100, cols)와 유사)
    public static double[][][] reshapeIMU(double[][] data, int segmentSize, int cols) {
        int numSegments = (int) Math.ceil((double) data.length / segmentSize);
        double[][][] reshaped = new double[numSegments][segmentSize][cols];

        for (int i = 0; i < numSegments; i++) {
            for (int j = 0; j < segmentSize; j++) {
                int dataIndex = i * segmentSize + j;
                reshaped[i][j] = dataIndex < data.length ? data[dataIndex] : new double[cols]; // 부족한 데이터 0으로 패딩
            }
        }
        return reshaped;
    }

    // ✅ 4. Timestamp 고유 값 추출 메서드 (Python의 DataFrame["timestamp"].unique()와 유사)
    public static Map<String, Object> getTimestampUnique(List<Map<String, Object>> imuData) {
        Set<Long> uniqueTimestamps = imuData.stream()
                .map(data -> (Long) data.get("timestamp"))
                .collect(Collectors.toSet());

        List<Long> sortedTimestamps = new ArrayList<>(uniqueTimestamps);
        Collections.sort(sortedTimestamps);

        Map<String, Object> timestampMap = new HashMap<>();
        timestampMap.put("timestamp", sortedTimestamps.toArray(new Long[0]));
        return timestampMap;
    }
}
