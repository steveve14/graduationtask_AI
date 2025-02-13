package com.example.movedistance;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class IMUProcessor {
    public static List<Map<String, Object>> preIMU(List<Map<String, Object>> imuData, String[] csvHeader) {
        if (imuData == null || imuData.isEmpty()) {
            throw new IllegalArgumentException("⚠ IMU 데이터가 비어 있습니다! CSV 파일을 확인하세요.");
        }

        System.out.println("✅ IMU Processing Started!");

        // ✅ 1. 헤더를 분석하여 센서 그룹 자동 탐색
        Map<String, List<String>> sensorGroups = IMUDataParser.parseSensorGroups(csvHeader);

        // ✅ 2. IMU 데이터를 저장할 맵 (각 센서별 3D 배열)
        Map<String, double[][][]> dfs = new HashMap<>();
        for (String sensor : sensorGroups.keySet()) {
            dfs.put(sensor, null);
        }

        // ✅ 3. IMU 데이터 처리
        for (String sensor : sensorGroups.keySet()) {
            List<String> sensorColumns = sensorGroups.get(sensor);
            double[][] cutIMU = IMUDataParser.cutIMU(sensor, sensorColumns, imuData);
            if (cutIMU.length == 0) {
                System.err.println("⚠ Warning: " + sensor + " 센서 데이터가 비어 있습니다. 기본값을 사용합니다.");
                cutIMU = new double[100][sensorColumns.size()];
            }

            System.out.println("📌 " + sensor + " 데이터 확인 (rows: " + cutIMU.length + ", cols: " + cutIMU[0].length + ")");

            double[][][] reshapedIMU = IMUDataParser.reshapeIMU(cutIMU, cutIMU.length / sensorColumns.size(), sensorColumns.size());
            dfs.put(sensor, reshapedIMU);
        }

        // ✅ 4. IMU 데이터 처리 (processingIMU 연결)
        List<Map<String, Object>> calcDFs = new ArrayList<>();
        calcDFs.add(IMUDataParser.getTimestampUnique(imuData));

        for (String sensor : sensorGroups.keySet()) {
            System.out.println("📌 Processing IMU Calculation: " + sensor);

            double[][][] sensorData = dfs.get(sensor);
            if (sensorData == null) {
                System.err.println("⚠ Warning: " + sensor + " 센서 데이터가 없습니다!");
                continue;
            }

            // **`processingIMU()` 호출**
            IMUResult imuResult = IMUFeatureExtractor.processingIMU(
                    sensorData,
                    sensorGroups.get(sensor).size(),
                    true, // stat_features
                    true, // spectral_features
                    null, // process
                    true, // process_each_axis
                    false, // calculate_jerk
                    dfs.get("rot"), // 회전 데이터
                    dfs.get("gravity"), // 중력 데이터
                    sensor
            );

            if (imuResult == null || imuResult.getFeatures() == null) {
                System.err.println("⚠ Warning: 처리된 데이터가 없습니다 - " + sensor);
                continue;
            }

            // 결과 저장
            Map<String, Object> processedData = new HashMap<>();
            processedData.put("sensor", sensor);
            processedData.put("processedData", imuResult.getFeatures());
            calcDFs.add(processedData);
        }

        return calcDFs;
    }

    // IMU 데이터 구조체
    public static class IMUResult {
        public double[][] x, y, z;
        public double[][] features;

        public IMUResult(double[][] x, double[][] y, double[][] z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public IMUResult(double[][] features) {
            this.features = features;
        }
        // `features` 값을 반환하는 메서드 추가
        public double[][] getFeatures() {
            return features;
        }

        public double[][] getX() {
            return x;
        }

        public double[][] getY() {
            return y;
        }

        public double[][] getZ() {
            return z;
        }
    }
}
