package com.example.movedistance;

import java.util.Arrays;
import org.apache.commons.math3.stat.StatUtils;
import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;
import org.apache.commons.math3.stat.descriptive.rank.Max;
import org.apache.commons.math3.stat.descriptive.rank.Min;

public class IMUFeatureExtractor {

    /**
     * IMU 센서 데이터(3D 혹은 4D 등)를 입력받아
     * 통계적/스펙트럼 피처를 추출한 2D 배열을 반환합니다.
     */
    public static double[][] processingIMU(
            double[][][] sensor,
            int numChannels,
            boolean statFeatures,
            boolean spectralFeatures,
            String process,
            boolean processEachAxis,
            boolean calculateJerk,
            double[][][] rotation,
            double[][][] gravity,
            String prefix) {

        int rows = sensor.length;
        int cols = sensor[0].length;

        System.out.println("📌 IMU Processing Started: " + prefix);
        System.out.println("📌 Sensor Data: rows=" + rows + ", cols=" + cols + ", numChannels=" + numChannels);

        // Quaternion 데이터 처리 (rot 센서)
        if ("rot".equals(prefix) && numChannels == 4) {
            System.out.println("✅ Quaternion detected. Returning raw data.");
            return extractRaw(sensor);
        }

        // 3채널 데이터: 각 축 추출 후 매그니튜드 계산
        double[][] x = extractChannel(sensor, 0);
        double[][] y = numChannels >= 3 ? extractChannel(sensor, 1) : new double[rows][cols];
        double[][] z = numChannels >= 3 ? extractChannel(sensor, 2) : new double[rows][cols];
        double[][] m = magnitude2D(x, y, z);

        // 특징 추출 비활성화 시 원본 데이터 반환
        if (!statFeatures && !spectralFeatures) {
            return mergeArrays(x, y, z, m);
        }

        // 회전 보정 (rotation 데이터 사용)
        if (rotation != null) {
            double[][][] rotated = rotateAxis(x, y, z, rotation);
            x = rotated[0];
            y = rotated[1];
            z = rotated[2];
            m = magnitude2D(x, y, z);
        }

        // 중력 벡터 사용 (수직/수평 성분 분리)
        if (gravity != null && process != null) {
            double[][] theta = calculateAngle(x, y, z, gravity);
            if ("horizontal".equals(process)) {
                System.out.println("✅ Calculating Horizontal Component...");
                m = multiplyMatrixElementWise(m, cosMatrix(theta));
            } else if ("vertical".equals(process)) {
                System.out.println("✅ Calculating Vertical Component...");
                m = multiplyMatrixElementWise(m, sinMatrix(theta));
            }
        }

        // Jerk(속도 변화율) 계산 (m, x, y, z)
        if (calculateJerk) {
            m = calculateJerk(m);
            x = calculateJerk(x);
            y = calculateJerk(y);
            z = calculateJerk(z);
        }

        // 통계적 특징 및 스펙트럼 특징 계산 (기본적으로 m 포함)
        double[][] statM = statFeatures ? calculateStatFeatures(m, prefix + "M") : null;
        double[][] spectralM = spectralFeatures ? calculateSpectralFeatures(m, 100, prefix + "M") : null;

        // 각 축별 추가 처리
        if (processEachAxis) {
            double[][] statX = statFeatures ? calculateStatFeatures(x, prefix + "X") : null;
            double[][] statY = statFeatures ? calculateStatFeatures(y, prefix + "Y") : null;
            double[][] statZ = statFeatures ? calculateStatFeatures(z, prefix + "Z") : null;
            statM = mergeFeatures(statM, statX, statY, statZ);

            double[][] spectralX = spectralFeatures ? calculateSpectralFeatures(x, 100,prefix + "X") : null;
            double[][] spectralY = spectralFeatures ? calculateSpectralFeatures(y, 100,prefix + "Y") : null;
            double[][] spectralZ = spectralFeatures ? calculateSpectralFeatures(z, 100,prefix + "Z") : null;
            spectralM = mergeFeatures(spectralM, spectralX, spectralY, spectralZ);
        }

        // 최종 반환 결정
        if (statFeatures && !spectralFeatures) {
            return statM;
        }
        if (!statFeatures) {
            return spectralM;
        }
        return mergeFeatures(statM, spectralM);
    }
    /**
     * 주어진 x, y, z 데이터와 중력 벡터(gravity)로부터 각도(theta)를 계산하여 2D 배열로 반환합니다.
     * lx, ly, lz: 센서의 x, y, z 채널 데이터 (행: 샘플, 열: 시점)
     * gravity: 중력 벡터 데이터 (3D 배열, 각 샘플에 대해 [gx, gy, gz])
     */
    public static double[][] calculateAngle(double[][] lx, double[][] ly, double[][] lz, double[][][] gravity) {
        int rows = lx.length;
        int cols = lx[0].length;

        double[][] gx = new double[rows][cols];
        double[][] gy = new double[rows][cols];
        double[][] gz = new double[rows][cols];

        // gravity 배열에서 gx, gy, gz 값을 추출
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                gx[i][j] = gravity[i][j][0];
                gy[i][j] = gravity[i][j][1];
                gz[i][j] = gravity[i][j][2];
            }
        }

        // NaN 값을 0으로 대체 (이미 클래스 내에 정의된 메서드를 사용)
        replaceNaNWithZero(gx);
        replaceNaNWithZero(gy);
        replaceNaNWithZero(gz);
        replaceNaNWithZero(lx);
        replaceNaNWithZero(ly);
        replaceNaNWithZero(lz);

        double[][] theta = new double[rows][cols];

        // 각 샘플별 각도 계산 (내적을 이용)
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                double dotProduct = gx[i][j] * lx[i][j] + gy[i][j] * ly[i][j] + gz[i][j] * lz[i][j];
                double magnitudeG = Math.sqrt(gx[i][j] * gx[i][j] + gy[i][j] * gy[i][j] + gz[i][j] * gz[i][j]);
                double magnitudeL = Math.sqrt(lx[i][j] * lx[i][j] + ly[i][j] * ly[i][j] + lz[i][j] * lz[i][j]);

                double denominator = magnitudeG * magnitudeL;
                if (denominator == 0) {
                    denominator = 0.000001; // 0으로 나누는 것을 방지
                }
                theta[i][j] = Math.acos(dotProduct / denominator);
            }
        }

        return theta;
    }


    // -------------------------
    // 통계적 피처 계산 관련 메서드
    // -------------------------
    public static double[][] calculateStatFeatures(double[][] magnitude, String prefix) {
        int rows = magnitude.length;
        double[] mean = new double[rows];
        double[] std = new double[rows];
        double[] max = new double[rows];
        double[] min = new double[rows];
        double[] mad = new double[rows];
        double[] iqr = new double[rows];

        for (int i = 0; i < rows; i++) {
            mean[i] = StatUtils.mean(magnitude[i]);
            std[i] = new StandardDeviation().evaluate(magnitude[i]);
            max[i] = new Max().evaluate(magnitude[i]);
            min[i] = new Min().evaluate(magnitude[i]);
            mad[i] = medianAbsoluteDeviation(magnitude[i]);
            iqr[i] = interquartileRange(magnitude[i]);
        }

        double[][] corr = autocorrelation(magnitude);
        double[] maxcorr = new double[rows];
        double[] argmaxCorr = new double[rows];
        double[] zcr = zeroCrossingRate(magnitude);
        int[] fzc = firstZeroCrossing(magnitude);

        for (int i = 0; i < rows; i++) {
            maxcorr[i] = Arrays.stream(corr[i]).max().orElse(0);
            argmaxCorr[i] = argMax(corr[i]);
        }

        // 최종 피처: 10개 (mean, std, max, min, mad, iqr, max.corr, idx.max.corr, zcr, fzc)
        double[][] result = new double[rows][10];
        for (int i = 0; i < rows; i++) {
            result[i][0] = mean[i];
            result[i][1] = std[i];
            result[i][2] = max[i];
            result[i][3] = min[i];
            result[i][4] = mad[i];
            result[i][5] = iqr[i];
            result[i][6] = maxcorr[i];
            result[i][7] = argmaxCorr[i];
            result[i][8] = zcr[i];
            result[i][9] = fzc[i];
        }
        return result;
    }

    // 중앙절대편차 (MAD)
    private static double medianAbsoluteDeviation(double[] data) {
        double median = StatUtils.percentile(data, 50);
        double[] deviations = Arrays.stream(data).map(d -> Math.abs(d - median)).toArray();
        return StatUtils.percentile(deviations, 50);
    }

    // 사분위 범위 (IQR)
    private static double interquartileRange(double[] data) {
        double q1 = StatUtils.percentile(data, 25);
        double q3 = StatUtils.percentile(data, 75);
        return q3 - q1;
    }

    // Autocorrelation 계산
    private static double[][] autocorrelation(double[][] data) {
        int rows = data.length;
        int cols = data[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            result[i] = computeAutocorrelation(data[i]);
        }
        return result;
    }

    private static double[] computeAutocorrelation(double[] x) {
        int n = x.length;
        double[] result = new double[n];
        for (int lag = 0; lag < n; lag++) {
            double sum = 0;
            for (int i = 0; i < n - lag; i++) {
                sum += x[i] * x[i + lag];
            }
            result[lag] = sum / (n - lag);
        }
        return result;
    }

    // Zero Crossing Rate (ZCR)
    public static double[] zeroCrossingRate(double[][] X) {
        int rows = X.length;
        double[] zcr = new double[rows];
        for (int i = 0; i < rows; i++) {
            zcr[i] = calculateZCR(X[i]);
        }
        return zcr;
    }

    private static double calculateZCR(double[] X) {
        int length = X.length;
        int S = 0;
        int prevIndex = 0;
        int[] temp = new int[length];
        for (int i = 0; i < length; i++) {
            temp[i] = (X[i] >= 0) ? 1 : -1;
        }
        for (int i = 0; i < length; i++) {
            prevIndex = i;
            if (i == 0) {
                continue;
            }
            S += Math.abs(temp[i] - temp[prevIndex]);
        }
        return 0.5 * S / length;
    }

    // First Zero Crossing (FZC)
    public static int[] firstZeroCrossing(double[][] X) {
        int rows = X.length;
        int[] fzc = new int[rows];
        for (int i = 0; i < rows; i++) {
            fzc[i] = calculateFZC(X[i]);
        }
        return fzc;
    }

    private static int calculateFZC(double[] X) {
        for (int i = 1; i < X.length; i++) {
            if (Math.signum(X[i]) != Math.signum(X[i - 1])) {
                return i;
            }
        }
        return 0;
    }

    // -------------------------
    // 스펙트럼 피처 계산 관련 메서드
    // -------------------------
    public static double[][] calculateSpectralFeatures(double[][] magnitude, int fs, String prefix) {
        int rows = magnitude.length;
        int cols = magnitude[0].length;

        System.out.println("📌 Spectral Features Calculation Started for " + prefix);
        System.out.println("📌 magnitude: rows=" + rows + ", cols=" + cols);

        if (cols < 2) {
            System.err.println("⚠ Warning: Spectral feature calculation skipped for " + prefix + ". Not enough data points.");
            return new double[rows][5];
        }

        double[][] PSD = computeWelchPSD(magnitude, fs);
        System.out.println("📌 PSD computed: rows=" + PSD.length + ", cols=" + (PSD.length > 0 ? PSD[0].length : "N/A"));

        if (PSD.length == 0 || PSD[0].length < 2) {
            System.err.println("⚠ Warning: Insufficient PSD data for " + prefix + ". Returning empty feature set.");
            return new double[rows][5];
        }

        double[][] FREQ = computeFrequencies(fs, PSD[0].length, PSD.length);
        System.out.println("📌 FREQ computed: rows=" + FREQ.length + ", cols=" + (FREQ.length > 0 ? FREQ[0].length : "N/A"));

        // 크기 불일치 확인
        for (int i = 0; i < rows; i++) {
            if (PSD[i].length != FREQ[i].length) {
                System.err.println("⚠ Warning: Row " + i + " has mismatched lengths (PSD=" + PSD[i].length + ", FREQ=" + FREQ[i].length + ")");
                return new double[rows][5];
            }
        }

        double[] maxPSD = findMax(PSD);
        double[] entropy = calculateEntropy(PSD);
        double[] fc = new double[rows];
        double[] sumPSD = new double[rows];

        for (int i = 0; i < rows; i++) {
            sumPSD[i] = Arrays.stream(PSD[i]).sum();
            if (sumPSD[i] == 0) sumPSD[i] = 0.000001;
            double weightedSum = 0;
            System.out.println("📌 Calculating FC for row " + i + " → PSD.length=" + PSD[i].length + ", FREQ.length=" + FREQ[i].length);
            for (int j = 0; j < Math.min(FREQ[i].length, PSD[i].length); j++) {
                weightedSum += FREQ[i][j] * PSD[i][j];
            }
            fc[i] = weightedSum / sumPSD[i];
        }

        double[] kurtosis = calculateKurtosis(PSD);
        double[] skewness = calculateSkewness(PSD);

        double[][] spectralFeatures = new double[rows][5];
        for (int i = 0; i < rows; i++) {
            spectralFeatures[i][0] = maxPSD[i];
            spectralFeatures[i][1] = entropy[i];
            spectralFeatures[i][2] = fc[i];
            spectralFeatures[i][3] = kurtosis[i];
            spectralFeatures[i][4] = skewness[i];
        }

        System.out.println("✅ Spectral Features Calculation Completed for " + prefix);
        return spectralFeatures;
    }

    // FFT, Welch PSD, FREQ, 및 기타 스펙트럼 관련 메서드
    private static double[][] computeWelchPSD(double[][] data, int fs) {
        int rows = data.length;
        int cols = data[0].length;

        System.out.println("📌 computeWelchPSD: rows=" + rows + ", cols=" + cols);

        if (cols < 2) {
            System.err.println("⚠ Warning: PSD computation skipped. Not enough data.");
            return new double[rows][1];
        }

        double[][] PSD = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            double[] fftResult = fft(data[i]);
            for (int j = 0; j < cols; j++) {
                PSD[i][j] = Math.pow(fftResult[j], 2) / fs;
            }
        }
        return PSD;
    }

    private static double[] fft(double[] data) {
        int n = data.length;
        double[] result = new double[n];
        for (int k = 0; k < n; k++) {
            double real = 0;
            double imag = 0;
            for (int t = 0; t < n; t++) {
                double angle = 2 * Math.PI * t * k / n;
                real += data[t] * Math.cos(angle);
                imag -= data[t] * Math.sin(angle);
            }
            result[k] = Math.sqrt(real * real + imag * imag);
        }
        return result;
    }

    private static double[][] computeFrequencies(int fs, int cols, int rows) {
        System.out.println("📌 computeFrequencies: cols=" + cols + ", rows=" + rows);
        if (cols < 2) {
            System.err.println("⚠ Warning: Frequency computation skipped. Not enough data.");
            return new double[rows][1];
        }
        double[][] freq = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                freq[i][j] = j * (double) fs / cols;
            }
        }
        return freq;
    }

    private static double[] findMax(double[][] data) {
        int rows = data.length;
        double[] maxVals = new double[rows];
        for (int i = 0; i < rows; i++) {
            maxVals[i] = Arrays.stream(data[i]).max().orElse(0);
        }
        return maxVals;
    }

    private static double[] calculateEntropy(double[][] data) {
        int rows = data.length;
        double[] entropy = new double[rows];
        for (int i = 0; i < rows; i++) {
            double sum = Arrays.stream(data[i]).sum();
            if (sum == 0) sum = 0.000001;
            entropy[i] = 0;
            for (double val : data[i]) {
                double prob = val / sum;
                if (prob > 0) {
                    entropy[i] -= prob * Math.log(prob);
                }
            }
        }
        return entropy;
    }

    private static double[] calculateKurtosis(double[][] data) {
        int rows = data.length;
        double[] kurtosis = new double[rows];
        for (int i = 0; i < rows; i++) {
            double mean = StatUtils.mean(data[i]);
            double std = new StandardDeviation().evaluate(data[i]);
            double sum = 0;
            for (double val : data[i]) {
                sum += Math.pow((val - mean) / std, 4);
            }
            kurtosis[i] = sum / data[i].length;
        }
        return kurtosis;
    }

    private static double[] calculateSkewness(double[][] data) {
        int rows = data.length;
        double[] skewness = new double[rows];
        for (int i = 0; i < rows; i++) {
            double mean = StatUtils.mean(data[i]);
            double std = new StandardDeviation().evaluate(data[i]);
            double sum = 0;
            for (double val : data[i]) {
                sum += Math.pow((val - mean) / std, 3);
            }
            skewness[i] = sum / data[i].length;
        }
        return skewness;
    }

    // -------------------------
    // 데이터 전처리 및 유틸리티 메서드
    // -------------------------
    private static double[][] extractRaw(double[][][] sensor) {
        int rows = sensor.length;
        int cols = sensor[0].length;
        double[][] rawData = new double[rows][cols * 4];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                rawData[i][j * 4] = sensor[i][j][0]; // w
                rawData[i][j * 4 + 1] = sensor[i][j][1]; // x
                rawData[i][j * 4 + 2] = sensor[i][j][2]; // y
                rawData[i][j * 4 + 3] = sensor[i][j][3]; // z
            }
        }
        return rawData;
    }


    private static double[][] extractChannel(double[][][] sensor, int index) {
        int rows = sensor.length;
        int cols = sensor[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            result[i] = sensor[i][index];
        }
        return result;
    }

    private static void replaceNaNWithZero(double[][] matrix) {
        for (int i = 0; i < matrix.length; i++) {
            for (int j = 0; j < matrix[i].length; j++) {
                if (Double.isNaN(matrix[i][j])) {
                    matrix[i][j] = 0.0;
                }
            }
        }
    }

    private static double[][] mergeArrays(double[][] x, double[][] y, double[][] z, double[][] m) {
        int rows = x.length;
        int cols = x[0].length * 4;
        double[][] merged = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            System.arraycopy(x[i], 0, merged[i], 0, x[i].length);
            System.arraycopy(y[i], 0, merged[i], x[i].length, y[i].length);
            System.arraycopy(z[i], 0, merged[i], x[i].length + y[i].length, z[i].length);
            System.arraycopy(m[i], 0, merged[i], x[i].length + y[i].length + z[i].length, m[i].length);
        }
        return merged;
    }

    private static double[][] multiplyMatrixElementWise(double[][] matrix1, double[][] matrix2) {
        int rows = matrix1.length;
        int cols = matrix1[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = matrix1[i][j] * matrix2[i][j];
            }
        }
        return result;
    }

    private static double[][] cosMatrix(double[][] matrix) {
        int rows = matrix.length;
        int cols = matrix[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = Math.cos(matrix[i][j]);
            }
        }
        return result;
    }

    private static double[][] sinMatrix(double[][] matrix) {
        int rows = matrix.length;
        int cols = matrix[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = Math.sin(matrix[i][j]);
            }
        }
        return result;
    }

    private static double[][] calculateJerk(double[][] data) {
        int rows = data.length;
        int cols = data[0].length - 1;
        double[][] jerk = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                jerk[i][j] = data[i][j + 1] - data[i][j];
            }
        }
        return jerk;
    }

    private static double[][][] rotateAxis(double[][] ax, double[][] ay, double[][] az, double[][][] rotation) {
        int rows = ax.length;
        int cols = ax[0].length;
        double[][] x = new double[rows][cols];
        double[][] y = new double[rows][cols];
        double[][] z = new double[rows][cols];
        double[][] w = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                x[i][j] = rotation[i][j][0];
                y[i][j] = rotation[i][j][1];
                z[i][j] = rotation[i][j][2];
                w[i][j] = rotation[i][j][3];
            }
        }

        double[] wFlat = flatten(w);
        double[] xFlat = flatten(x);
        double[] yFlat = flatten(y);
        double[] zFlat = flatten(z);
        double[] axFlat = flatten(ax);
        double[] ayFlat = flatten(ay);
        double[] azFlat = flatten(az);
        double[] _ax = new double[axFlat.length];
        double[] _ay = new double[ayFlat.length];
        double[] _az = new double[azFlat.length];

        for (int i = 0; i < axFlat.length; i++) {
            _ax[i] = (1 - 2 * (yFlat[i] * yFlat[i] + zFlat[i] * zFlat[i])) * axFlat[i]
                    + 2 * (xFlat[i] * yFlat[i] - wFlat[i] * zFlat[i]) * ayFlat[i]
                    + 2 * (xFlat[i] * zFlat[i] + wFlat[i] * yFlat[i]) * azFlat[i];

            _ay[i] = 2 * (xFlat[i] * yFlat[i] + wFlat[i] * zFlat[i]) * axFlat[i]
                    + (1 - 2 * (xFlat[i] * xFlat[i] + zFlat[i] * zFlat[i])) * ayFlat[i]
                    + 2 * (yFlat[i] * zFlat[i] - wFlat[i] * xFlat[i]) * azFlat[i];

            _az[i] = 2 * (xFlat[i] * zFlat[i] - wFlat[i] * yFlat[i]) * axFlat[i]
                    + 2 * (yFlat[i] * zFlat[i] + wFlat[i] * xFlat[i]) * ayFlat[i]
                    + (1 - 2 * (xFlat[i] * xFlat[i] + yFlat[i] * yFlat[i])) * azFlat[i];
        }

        return new double[][][] {
                reshape(_ax, rows, cols),
                reshape(_ay, rows, cols),
                reshape(_az, rows, cols)
        };
    }

    private static double[] flatten(double[][] matrix) {
        return Arrays.stream(matrix).flatMapToDouble(Arrays::stream).toArray();
    }

    private static double[][] reshape(double[] array, int rows, int cols) {
        double[][] reshaped = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            System.arraycopy(array, i * cols, reshaped[i], 0, cols);
        }
        return reshaped;
    }

    private static int argMax(double[] array) {
        int maxIndex = 0;
        double maxVal = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] > maxVal) {
                maxVal = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    public static double[] magnitude(double[] x, double[] y, double[] z) {
        int length = x.length;
        double[] result = new double[length];
        for (int i = 0; i < length; i++) {
            result[i] = Math.sqrt(x[i] * x[i] + y[i] * y[i] + z[i] * z[i]);
        }
        return result;
    }

    private static double[][] magnitude2D(double[][] x, double[][] y, double[][] z) {
        int rows = x.length;
        int cols = x[0].length;
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = Math.sqrt(x[i][j] * x[i][j] + y[i][j] * y[i][j] + z[i][j] * z[i][j]);
            }
        }
        return result;
    }


    private static double[][] mergeFeatures(double[][]... features) {
        if (features == null || features.length == 0) {
            return null;
        }
        int rows = 0;
        for (double[][] feature : features) {
            if (feature != null) {
                rows = feature.length;
                break;
            }
        }
        if (rows == 0) return null;
        int totalCols = 0;
        for (double[][] feature : features) {
            if (feature != null) {
                totalCols += feature[0].length;
            }
        }
        double[][] merged = new double[rows][totalCols];
        int colIndex = 0;
        for (double[][] feature : features) {
            if (feature != null) {
                for (int i = 0; i < rows; i++) {
                    System.arraycopy(feature[i], 0, merged[i], colIndex, feature[i].length);
                }
                colIndex += feature[0].length;
            }
        }
        return merged;
    }

    // -------------------------
    // 헤더 생성 메서드 추가
    // -------------------------
    /**
     * 통계적 피처에 대한 헤더 문자열을 생성합니다.
     * 예: prefix가 "accelM"이면 "accelM_mean	accelM_std	accelM_max	accelM_min	accelM_mad	accelM_iqr	accelM_max.corr	accelM_idx.max.corr	accelM_zcr	accelM_fzc"를 반환.
     */
    public static String getStatFeatureHeader(String prefix) {
        String[] features = {
                prefix + "_mean",
                prefix + "_std",
                prefix + "_max",
                prefix + "_min",
                prefix + "_mad",
                prefix + "_iqr",
                prefix + "_max.corr",
                prefix + "_idx.max.corr",
                prefix + "_zcr",
                prefix + "_fzc"
        };
        return String.join("\t", features);
    }

    /**
     * 스펙트럼 피처에 대한 헤더 문자열을 생성합니다.
     * 예: prefix가 "accelM"이면 "accelM_max.psd	accelM_entropy	accelM_fc	accelM_kurt	accelM_skew"를 반환.
     */
    public static String getSpectralFeatureHeader(String prefix) {
        String[] features = {
                prefix + "_max.psd",
                prefix + "_entropy",
                prefix + "_fc",
                prefix + "_kurt",
                prefix + "_skew"
        };
        return String.join("\t", features);
    }

    /**
     * 3D 센서 데이터의 통계적 피처 헤더를 생성합니다.
     * 예: sensorPrefix가 "accel"이면 accelM, accelX, accelY, accelZ에 대한 헤더를 모두 결합하여 반환합니다.
     */
    public static String get3DAxesStatFeatureHeader(String sensorPrefix) {
        String headerM = getStatFeatureHeader(sensorPrefix + "M");
        String headerX = getStatFeatureHeader(sensorPrefix + "X");
        String headerY = getStatFeatureHeader(sensorPrefix + "Y");
        String headerZ = getStatFeatureHeader(sensorPrefix + "Z");
        return headerM + "\t" + headerX + "\t" + headerY + "\t" + headerZ;
    }

    /**
     * 3D 센서 데이터의 스펙트럼 피처 헤더를 생성합니다.
     */
    public static String get3DAxesSpectralFeatureHeader(String sensorPrefix) {
        String headerM = getSpectralFeatureHeader(sensorPrefix + "M");
        String headerX = getSpectralFeatureHeader(sensorPrefix + "X");
        String headerY = getSpectralFeatureHeader(sensorPrefix + "Y");
        String headerZ = getSpectralFeatureHeader(sensorPrefix + "Z");
        return headerM + "\t" + headerX + "\t" + headerY + "\t" + headerZ;
    }

    // -------------------------
    // 메인 메서드 (테스트용)
    // -------------------------
    public static void main(String[] args) {
        // 예시: accel 센서의 3D 데이터 샘플
        double[][] accelData = {
                {0.1, 0.2, 9.7},
                {0.0, 0.3, 9.8},
                {-0.1, 0.1, 9.6},
                {0.05, 0.25, 9.75},
                {0.2, 0.15, 9.65}
        };
        // 통계적 피처 헤더 생성 예시
        String statHeader = get3DAxesStatFeatureHeader("accel");
        System.out.println("Stat Feature Header:");
        System.out.println(statHeader);

        // 스펙트럼 피처 헤더 생성 예시
        String spectralHeader = get3DAxesSpectralFeatureHeader("accel");
        System.out.println("\nSpectral Feature Header:");
        System.out.println(spectralHeader);
    }
}
