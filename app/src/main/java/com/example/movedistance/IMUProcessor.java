package com.example.movedistance;

import java.util.*;

public class IMUProcessor {
    // 센서 그룹 정의
    private static final Map<String, Integer> sensorChannels = new HashMap<>();

    static {
        sensorChannels.put("accel", 3);
        sensorChannels.put("gyro", 3);
        sensorChannels.put("mag", 3);
        sensorChannels.put("gravity", 3);
        sensorChannels.put("linear_accel", 3);
        sensorChannels.put("rot", 4);
    }

    public static Map<String, double[][]> processIMU(List<Map<String, Object>> imuData) {
        if (imuData == null || imuData.isEmpty()) {
            throw new IllegalArgumentException("⚠ IMU 데이터가 비어 있습니다! CSV 파일을 확인하세요.");
        }

        System.out.println("✅ IMU Processing Started!");

        // ✅ 1. 센서 데이터를 저장할 맵 (각 센서별 3D 배열)
        Map<String, double[][][]> sensorDataMap = new HashMap<>();
        for (String sensor : sensorChannels.keySet()) {
            sensorDataMap.put(sensor, null);
        }

        // ✅ 2. 센서별로 데이터 변환 (Python의 cut_imu() 및 reshape()와 유사)
        for (String sensor : sensorChannels.keySet()) {
            int numChannels = sensorChannels.get(sensor);
            double[][] cutIMU = IMUDataParser.cutIMU(sensor, numChannels, imuData);

            if (cutIMU.length == 0) {
                System.err.println("⚠ Warning: " + sensor + " 센서 데이터가 비어 있습니다. 기본값을 사용합니다.");
                cutIMU = new double[100][numChannels];
            }

            double[][][] reshapedIMU = IMUDataParser.reshapeIMU(cutIMU, cutIMU.length / numChannels, numChannels);
            sensorDataMap.put(sensor, reshapedIMU);
        }

        // ✅ 3. IMU 특징 추출 (Python의 processing_imu() 함수와 동일한 흐름)
        Map<String, double[][]> processedFeatures = new HashMap<>();
        for (String sensor : sensorChannels.keySet()) {
            double[][][] sensorData = sensorDataMap.get(sensor);
            if (sensorData == null) {
                System.err.println("⚠ Warning: " + sensor + " 센서 데이터가 없습니다!");
                continue;
            }

            // 특징 추출 수행
            double[][] featureData = IMUFeatureExtractor.processingIMU(
                    sensorData,
                    sensorChannels.get(sensor),
                    true,  // stat_features
                    true,  // spectral_features
                    null,  // process
                    true,  // process_each_axis
                    false, // calculate_jerk
                    sensorDataMap.get("rot"),  // 회전 데이터
                    sensorDataMap.get("gravity"),  // 중력 데이터
                    sensor
            );

            if (featureData == null) {
                System.err.println("⚠ Warning: " + sensor + " 처리된 데이터가 없습니다!");
                continue;
            }

            processedFeatures.put(sensor, featureData);
        }

        System.out.println("✅ Final Processed IMU Features: " + processedFeatures.keySet());
        return processedFeatures;
    }
}
