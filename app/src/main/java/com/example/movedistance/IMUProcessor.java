package com.example.movedistance;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class IMUProcessor {
    // 센서 그룹 정의 (각 센서별 채널 수)
    private static final Map<String, Integer> sensorChannels = new HashMap<>();

    static {
        sensorChannels.put("accel", 3);
        sensorChannels.put("gyro", 3);
        sensorChannels.put("mag", 3);
        sensorChannels.put("gravity", 3);
        sensorChannels.put("linear_accel", 3);
        sensorChannels.put("rot", 4);
    }

    /**
     * 개별 센서별로 특징을 추출하여 Map<String, double[][]>로 반환합니다.
     */
    public static Map<String, double[][]> processIMU(List<Map<String, Object>> imuData) {
        if (imuData == null || imuData.isEmpty()) {
            throw new IllegalArgumentException("⚠ IMU 데이터가 비어 있습니다! CSV 파일을 확인하세요.");
        }

        System.out.println("✅ IMU Processing Started!");

        // 1. 센서별 원시 데이터를 저장할 맵 (각 센서별 3D 배열)
        Map<String, double[][][]> sensorDataMap = new HashMap<>();
        for (String sensor : sensorChannels.keySet()) {
            sensorDataMap.put(sensor, null);
        }

        // 2. 각 센서별로 데이터를 변환 (cut_imu 및 reshape)
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

        // 3. 각 센서별 특징 추출
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
                    true,     // stat_features 활성화
                    true,     // spectral_features 활성화
                    null,     // process 옵션 (예: "horizontal" 또는 "vertical" 등)
                    true,     // process_each_axis 활성화
                    false,    // calculate_jerk 비활성화
                    sensorDataMap.get("rot"),     // 회전 데이터
                    sensorDataMap.get("gravity"), // 중력 데이터
                    sensor    // prefix (예: "accel", "gyro", …)
            );

            if (featureData == null) {
                System.err.println("⚠ Warning: " + sensor + " 처리된 데이터가 없습니다!");
                continue;
            }

            processedFeatures.put(sensor, featureData);
        }

        System.out.println("✅ Final Processed IMU Features (per sensor): " + processedFeatures.keySet());

        // 각 센서별 피처 행렬의 크기를 출력
        printSensorFeaturesInfo(processedFeatures);

        return processedFeatures;
    }

    /**
     * processIMU()로 개별 센서별 특징을 추출한 후, 모두 병합하여 하나의 특징 행렬로 반환합니다.
     * 예: 각 센서별로 60개의 피처가 추출되었다면 병합 후 전체 피처 행렬은 3행 x (6*60)열이 됩니다.
     */
    public static double[][] processAndMergeIMU(List<Map<String, Object>> imuData) {
        Map<String, double[][]> sensorFeatures = processIMU(imuData);
        double[][] mergedFeatures = mergeSensorFeatures(sensorFeatures);
        System.out.println("✅ Merged IMU Feature Shape: " + mergedFeatures.length + " x " + mergedFeatures[0].length);

        // 예상 피처 개수를 확인 (예: 각 센서별 헤더 수를 알고 있다면)
        // 예를 들어, accel의 stat 피처 헤더를 가져와 예상 개수를 계산한다고 가정
        String accelStatHeader = IMUFeatureExtractor.get3DAxesStatFeatureHeader("accel");
        int expectedPerSensor = accelStatHeader.split("\t").length; // accelM, accelX, accelY, accelZ 모두 합산된 수
        int totalExpected = 0;
        // sensorChannels에 포함된 센서 중 "rot" 센서는 별도 처리되므로 제외하거나 다른 방식으로 계산할 수 있습니다.
        // 여기서는 rot를 제외하고 나머지 5개 센서에 대해 계산한다고 가정합니다.
        totalExpected = expectedPerSensor * 5;
        System.out.println("예상 전체 피처 개수 (rot 제외): " + totalExpected);
        if (mergedFeatures[0].length != totalExpected) {
            System.err.println("⚠ 오류: 병합된 피처 개수(" + mergedFeatures[0].length + ")가 예상(" + totalExpected + ")과 다릅니다.");
        } else {
            System.out.println("✅ 병합된 피처 개수가 예상과 일치합니다.");
        }
        return mergedFeatures;
    }

    /**
     * 센서별 특징 행렬들을 열 방향으로 병합합니다.
     * 모든 센서의 특징 행렬은 동일한 행 수(동일한 시간 창 수)를 가지는 것으로 가정합니다.
     */
    private static double[][] mergeSensorFeatures(Map<String, double[][]> featuresMap) {
        int rows = -1;
        int totalCols = 0;
        for (double[][] matrix : featuresMap.values()) {
            if (matrix != null) {
                if (rows == -1) {
                    rows = matrix.length;
                }
                totalCols += matrix[0].length;
            }
        }
        if (rows == -1) {
            throw new IllegalStateException("병합할 특징 행렬이 없습니다.");
        }
        double[][] merged = new double[rows][totalCols];
        int colIndex = 0;
        for (String key : featuresMap.keySet()) {
            double[][] matrix = featuresMap.get(key);
            if (matrix != null) {
                for (int i = 0; i < rows; i++) {
                    System.arraycopy(matrix[i], 0, merged[i], colIndex, matrix[i].length);
                }
                colIndex += matrix[0].length;
            }
        }
        return merged;
    }

    /**
     * 각 센서별로 추출된 특징 행렬의 크기(행, 열)를 출력합니다.
     */
    private static void printSensorFeaturesInfo(Map<String, double[][]> featuresMap) {
        for (Map.Entry<String, double[][]> entry : featuresMap.entrySet()) {
            String sensor = entry.getKey();
            double[][] featureMatrix = entry.getValue();
            int rows = featureMatrix.length;
            int cols = (rows > 0) ? featureMatrix[0].length : 0;
            System.out.println(sensor + " 센서: " + rows + "행, " + cols + "열");
        }
    }
}
