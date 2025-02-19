package com.example.movedistance;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.telephony.CellIdentityLte;
import android.telephony.CellInfo;
import android.telephony.CellInfoLte;
import android.telephony.TelephonyManager;
import android.util.Log;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationServices;
import org.pytorch.Tensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class SensorDataCollector extends AppCompatActivity {
    private static final int PROCESS_INTERVAL_60 = 60000; // 60초 간격
    private static final int PROCESS_INTERVAL_01 = 1000;  // 1초 간격
    private static final int PERMISSION_REQUEST_CODE = 1;

    // 기본 센서 데이터는 최대 60개, IMU 데이터는 최대 60000개 저장하도록 설정
    private static final int MAX_SIZE_DEFAULT = 60;
    private static final int MAX_SIZE_IMU = 60000;

    private final Handler handler = new Handler(Looper.getMainLooper());

    // AP (WiFi)
    private final List<Map<String, Object>> apDataList = new ArrayList<>();
    private final List<Map<String, Object>> apProcessedDataList = new ArrayList<>();

    // BTS
    private final List<Map<String, Object>> btsDataList = new ArrayList<>();
    private final List<Map<String, Object>> btsProcessedDataList = new ArrayList<>();

    // GPS
    private final List<Map<String, Object>> gpsDataList = new ArrayList<>();
    private final List<Map<String, Object>> gpsProcessedDataList = new ArrayList<>();

    // IMU
    private final List<Map<String, Object>> imuDataList = new ArrayList<>();
    private final List<Map<String, Object>> imuProcessedDataList = new ArrayList<>();

    private WifiManager wifiManager;
    private TelephonyManager telephonyManager;
    private FusedLocationProviderClient fusedLocationProviderClient;
    private SensorManager sensorManager;
    private PyTorchHelper pyTorchHelper;
    private final ExecutorService executorService = Executors.newSingleThreadExecutor();
    private boolean notstartAI = true;

    TextView textAP, textBTS, textGPS, textIMU, textAIResult, textViewProcessed;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_ai);

        textAP = findViewById(R.id.textAP);
        textBTS = findViewById(R.id.textBTS);
        textGPS = findViewById(R.id.textGPS);
        textIMU = findViewById(R.id.textIMU);
        textAIResult = findViewById(R.id.textViewAI);
        textViewProcessed = findViewById(R.id.textViewProcessed);

        wifiManager = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        telephonyManager = (TelephonyManager) getSystemService(Context.TELEPHONY_SERVICE);
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        fusedLocationProviderClient = LocationServices.getFusedLocationProviderClient(this);

        requestPermissions();
    }

    private void requestPermissions() {
        if (!hasPermissions()) {
            ActivityCompat.requestPermissions(this, new String[]{
                    Manifest.permission.ACCESS_FINE_LOCATION,
                    Manifest.permission.ACCESS_WIFI_STATE,
                    Manifest.permission.READ_PHONE_STATE
            }, PERMISSION_REQUEST_CODE);
        } else {
            startDataCollection();
        }
    }

    private boolean hasPermissions() {
        return ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED &&
                ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_WIFI_STATE) == PackageManager.PERMISSION_GRANTED &&
                ContextCompat.checkSelfPermission(this, Manifest.permission.READ_PHONE_STATE) == PackageManager.PERMISSION_GRANTED;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == PERMISSION_REQUEST_CODE) {
            boolean allGranted = true;
            for (int result : grantResults) {
                if (result != PackageManager.PERMISSION_GRANTED) {
                    allGranted = false;
                    break;
                }
            }
            if (allGranted) {
                startDataCollection();
            } else {
                Log.e("Permissions", "필수 권한이 거부됨. 앱 기능이 제한됩니다.");
            }
        }
    }

    private void startDataCollection() {
        // 1초마다 AP, BTS, GPS 데이터 단발 수집 (IMU는 누적 수집으로 별도 진행)
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                long timestamp = System.currentTimeMillis();
                collectAPData(timestamp);
                collectBTSData(timestamp);
                collectGPSData(timestamp);
                // 단발성 IMU 수집은 제거
                handler.postDelayed(this, PROCESS_INTERVAL_01);
            }
        }, PROCESS_INTERVAL_01);

        // 60초마다 AP, BTS, GPS, AI 데이터 처리
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                textViewProcessed.setText("");
                processAPData();
                processBTSData();
                processGPSData();
                processAI();
                handler.postDelayed(this, PROCESS_INTERVAL_60);
            }
        }, PROCESS_INTERVAL_60);

        // 별도 IMU 누적 수집 사이클 시작 (1분 주기)
        startIMUAccumulationCycle();
    }

    private void startIMUAccumulationCycle() {
        imuDataList.clear(); // 기존 누적 데이터 초기화
        collectIMUDataAccumulated();

        // 60초 후 (60초 + 1초 마진) 누적된 IMU 데이터를 처리 후 다음 사이클 재시작
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                processIMUData();
                startIMUAccumulationCycle();
            }
        }, PROCESS_INTERVAL_60 + 1000);
    }

    // AP 데이터 수집
    private void collectAPData(long timestamp) {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_WIFI_STATE)
                == PackageManager.PERMISSION_GRANTED) {
            try {
                if (wifiManager != null) {
                    List<ScanResult> scanResults = wifiManager.getScanResults();
                    if (!scanResults.isEmpty()) {
                        ScanResult scanResult = scanResults.get(0);
                        String ssid = "UNKNOWN_SSID";
                        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                                == PackageManager.PERMISSION_GRANTED) {
                            WifiInfo wifiInfo = wifiManager.getConnectionInfo();
                            ssid = wifiInfo.getSSID();
                        } else {
                            Log.e("WiFi", "WiFi SSID 가져오기 실패: 위치 권한이 없음");
                        }
                        Map<String, Object> data = new HashMap<>();
                        data.put("timestamp", timestamp);
                        data.put("wifibssid", scanResult.BSSID);
                        data.put("ssid", ssid);
                        data.put("level", scanResult.level);
                        data.put("frequency", scanResult.frequency);
                        data.put("capabilities", scanResult.capabilities);
                        addData(apDataList, data, "WiFi AP");
                        textAP.setText(data.toString());
                    }
                }
            } catch (SecurityException e) {
                Log.e("WiFi", "Wi-Fi 데이터 수집 실패: 권한이 부족함", e);
            }
        }
    }

    // AP 데이터 처리
    private void processAPData() {
        if (!apDataList.isEmpty()) {
            long startTimestamp = System.currentTimeMillis() - (60 * 1000);
            List<Map<String, Object>> processedData = APProcessor.processAP(apDataList, startTimestamp);
            textViewProcessed.append("AP:" + processedData.size());
            for (Map<String, Object> entry : processedData) {
                addData(apProcessedDataList, entry, "WiFi Processed");
            }
        }
    }

    // BTS 데이터 수집
    private void collectBTSData(long timestamp) {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.READ_PHONE_STATE)
                == PackageManager.PERMISSION_GRANTED) {
            try {
                if (telephonyManager != null) {
                    List<CellInfo> cellInfoList = telephonyManager.getAllCellInfo();
                    for (CellInfo cellInfo : cellInfoList) {
                        if (cellInfo instanceof CellInfoLte) {
                            CellIdentityLte cellIdentity = ((CellInfoLte) cellInfo).getCellIdentity();
                            Map<String, Object> data = new HashMap<>();
                            data.put("timestamp", timestamp);
                            data.put("ci", cellIdentity.getCi());
                            data.put("pci", cellIdentity.getPci());
                            addData(btsDataList, data, "BTS Raw");
                            textBTS.setText(data.toString());
                        }
                    }
                }
            } catch (SecurityException e) {
                Log.e("BTS", "BTS 데이터 수집 실패: 권한이 부족함", e);
            }
        }
    }

    // BTS 데이터 처리
    private void processBTSData() {
        if (!btsDataList.isEmpty()) {
            long startTimestamp = System.currentTimeMillis() - (60 * 1000);
            List<Map<String, Object>> processedData = BTSProcessor.processBTS(btsDataList, startTimestamp);
            textViewProcessed.append(", BTS:" + processedData.size() + "\n");
            for (Map<String, Object> entry : processedData) {
                addData(btsProcessedDataList, entry, "BTS Processed");
            }
        }
    }

    // GPS 데이터 수집
    private void collectGPSData(long timestamp) {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED) {
            fusedLocationProviderClient.getLastLocation().addOnSuccessListener(location -> {
                if (location != null) {
                    Map<String, Object> data = new HashMap<>();
                    data.put("timestamp", timestamp);
                    data.put("latitude", location.getLatitude());
                    data.put("longitude", location.getLongitude());
                    addData(gpsDataList, data, "GPS Raw");
                    textGPS.setText(data.toString());
                }
            });
        } else {
            Log.e("GPS", "GPS permission is missing.");
        }
    }

    // GPS 데이터 처리
    private void processGPSData() {
        if (!gpsDataList.isEmpty()) {
            long startTimestamp = System.currentTimeMillis() - (60 * 1000);
            List<Map<String, Object>> processedData = GPSProcessor.processGPS(gpsDataList, startTimestamp);
            textViewProcessed.append("GPS:" + processedData.size());
            for (Map<String, Object> entry : processedData) {
                addData(gpsProcessedDataList, entry, "GPS Processed");
            }
        }
    }

    // IMU 데이터 누적 수집
    private void collectIMUDataAccumulated() {
        if (sensorManager == null) {
            Log.e("IMU", "SensorManager is not initialized.");
            return;
        }

        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor rotationVector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        Sensor pressureSensor = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
        Sensor gravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        Sensor linearAccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        if (accelerometer == null || gyroscope == null || magnetometer == null ||
                rotationVector == null || pressureSensor == null ||
                gravitySensor == null || linearAccelSensor == null) {
            Log.e("IMU", "One or more sensors are not available on this device.");
            return;
        }

        final Map<Integer, float[]> sensorValues = new ConcurrentHashMap<>();

        SensorEventListener accumulationListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                sensorValues.put(event.sensor.getType(), event.values.clone());
            }
            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) { }
        };

        sensorManager.registerListener(accumulationListener, accelerometer, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(accumulationListener, gyroscope, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(accumulationListener, magnetometer, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(accumulationListener, rotationVector, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(accumulationListener, pressureSensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(accumulationListener, gravitySensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(accumulationListener, linearAccelSensor, SensorManager.SENSOR_DELAY_GAME);

        final long accumulationDuration = PROCESS_INTERVAL_60; // 60초 누적
        final long startTime = System.currentTimeMillis();

        // 10ms 간격으로 센서 데이터를 기록
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (sensorValues.containsKey(Sensor.TYPE_ACCELEROMETER) &&
                        sensorValues.containsKey(Sensor.TYPE_GYROSCOPE) &&
                        sensorValues.containsKey(Sensor.TYPE_MAGNETIC_FIELD) &&
                        sensorValues.containsKey(Sensor.TYPE_ROTATION_VECTOR) &&
                        sensorValues.containsKey(Sensor.TYPE_PRESSURE) &&
                        sensorValues.containsKey(Sensor.TYPE_GRAVITY) &&
                        sensorValues.containsKey(Sensor.TYPE_LINEAR_ACCELERATION)) {

                    Map<String, Object> data = new HashMap<>();
                    data.put("timestamp", System.currentTimeMillis());

                    float[] accel = sensorValues.get(Sensor.TYPE_ACCELEROMETER);
                    data.put("accel.x", accel[0]);
                    data.put("accel.y", accel[1]);
                    data.put("accel.z", accel[2]);

                    float[] gyro = sensorValues.get(Sensor.TYPE_GYROSCOPE);
                    data.put("gyro.x", gyro[0]);
                    data.put("gyro.y", gyro[1]);
                    data.put("gyro.z", gyro[2]);

                    float[] mag = sensorValues.get(Sensor.TYPE_MAGNETIC_FIELD);
                    data.put("mag.x", mag[0]);
                    data.put("mag.y", mag[1]);
                    data.put("mag.z", mag[2]);

                    float[] rot = sensorValues.get(Sensor.TYPE_ROTATION_VECTOR);
                    float[] quat = new float[4];
                    SensorManager.getQuaternionFromVector(quat, rot);
                    data.put("rot.w", quat[0]);
                    data.put("rot.x", quat[1]);
                    data.put("rot.y", quat[2]);
                    data.put("rot.z", quat[3]);

                    float[] pressure = sensorValues.get(Sensor.TYPE_PRESSURE);
                    data.put("pressure.x", pressure[0]);

                    float[] gravity = sensorValues.get(Sensor.TYPE_GRAVITY);
                    data.put("gravity.x", gravity[0]);
                    data.put("gravity.y", gravity[1]);
                    data.put("gravity.z", gravity[2]);

                    float[] linear = sensorValues.get(Sensor.TYPE_LINEAR_ACCELERATION);
                    data.put("linear_accel.x", linear[0]);
                    data.put("linear_accel.y", linear[1]);
                    data.put("linear_accel.z", linear[2]);

                    addData(imuDataList, data, "IMU Accumulated");
                    textIMU.setText("IMU 누적 데이터 수: " + imuDataList.size());
                }

                if (System.currentTimeMillis() - startTime < accumulationDuration) {
                    handler.postDelayed(this, 10);
                } else {
                    sensorManager.unregisterListener(accumulationListener);
                    Log.d("IMU", "누적 수집 종료");
                }
            }
        }, 10);
    }

    // IMU 데이터 처리 (누적 데이터 기반)
    private void processIMUData() {
        if (!imuDataList.isEmpty()) {
            long startTimestamp = System.currentTimeMillis() - (60 * 1000);
            Map<String, double[][]> processedData = IMUProcessor.processIMU(imuDataList);
            textViewProcessed.append(", IMU:" + processedData.size());
            for (Map.Entry<String, double[][]> entry : processedData.entrySet()) {
                Map<String, Object> dataMap = new HashMap<>();
                dataMap.put(entry.getKey(), entry.getValue());
                addData(imuProcessedDataList, dataMap, "IMU Processed");
            }
        }
    }

    // AI 처리 - 여기서 AP, BTS, GPS, IMU 순서로 병합된 피처 벡터를 AI 모델의 입력으로 사용합니다.
    private void processAI() {
        executorService.execute(() -> {
            if (pyTorchHelper == null) {
                Log.e("AI", "PyTorchHelper 초기화");
                pyTorchHelper = new PyTorchHelper(this);
                runOnUiThread(() -> textAIResult.setText("PyTorchHelper 초기화 완료"));
            }

            if (notstartAI) {
                notstartAI = false;
            } else {
                // AP, BTS, GPS, IMU 처리된 데이터 병합하여 하나의 피처 벡터 생성
                float[] inputFeatureVector = getMergedFeatureVector();
                if (inputFeatureVector == null) {
                    Log.e("AI", "합쳐진 피처 벡터 생성 실패");
                    return;
                }
                // Tensor 생성 (여기서는 단일 시간 창으로 가정)
                Tensor inputTensor = Tensor.fromBlob(inputFeatureVector, new long[]{1, 1, inputFeatureVector.length});
                float[] outputData = pyTorchHelper.predict(inputTensor);
                Log.d("PyTorch Output", "Result: " + Arrays.toString(outputData));
                runOnUiThread(() -> textAIResult.setText(Arrays.toString(outputData)));
            }
        });
    }

    /**
     * AP, BTS, GPS, IMU 각각의 처리된 데이터 리스트에서 가장 최근의 데이터를 가져와
     * (각각 Map<String, Object> 형태로 저장됨)
     * 그 안에 저장된 double[][](feature 행렬)의 첫 행을 추출한 후, float 배열로 변환하여 순서대로 병합합니다.
     */
    private float[] getMergedFeatureVector() {
        if (apProcessedDataList.isEmpty() || btsProcessedDataList.isEmpty()
                || gpsProcessedDataList.isEmpty() || imuProcessedDataList.isEmpty()) {
            Log.e("AI", "Processed data 중 일부가 없습니다.");
            return null;
        }
        Map<String, Object> apMap = apProcessedDataList.get(apProcessedDataList.size() - 1);
        Map<String, Object> btsMap = btsProcessedDataList.get(btsProcessedDataList.size() - 1);
        Map<String, Object> gpsMap = gpsProcessedDataList.get(gpsProcessedDataList.size() - 1);
        Map<String, Object> imuMap = imuProcessedDataList.get(imuProcessedDataList.size() - 1);

        float[] apFeatures = extractFeatureVectorFromMap(apMap);
        float[] btsFeatures = extractFeatureVectorFromMap(btsMap);
        float[] gpsFeatures = extractFeatureVectorFromMap(gpsMap);
        float[] imuFeatures = extractFeatureVectorFromMap(imuMap);

        if (apFeatures == null || btsFeatures == null || gpsFeatures == null || imuFeatures == null) {
            Log.e("AI", "Some sensor features are null");
            return null;
        }

        int totalLength = apFeatures.length + btsFeatures.length + gpsFeatures.length + imuFeatures.length;
        float[] merged = new float[totalLength];
        int pos = 0;
        System.arraycopy(apFeatures, 0, merged, pos, apFeatures.length);
        pos += apFeatures.length;
        System.arraycopy(btsFeatures, 0, merged, pos, btsFeatures.length);
        pos += btsFeatures.length;
        System.arraycopy(gpsFeatures, 0, merged, pos, gpsFeatures.length);
        pos += gpsFeatures.length;
        System.arraycopy(imuFeatures, 0, merged, pos, imuFeatures.length);
        return merged;
    }

    /**
     * Map 안에 저장된 feature 행렬(double[][])의 첫 행을 추출하여 float[]로 변환합니다.
     * Map은 단일 entry를 가지고 있다고 가정합니다.
     */
    private float[] extractFeatureVectorFromMap(Map<String, Object> map) {
        if (map == null || map.isEmpty()) return null;
        Object value = map.values().iterator().next();
        if (!(value instanceof double[][])) {
            Log.e("AI", "Feature value is not a double[][]");
            return null;
        }
        double[][] matrix = (double[][]) value;
        if (matrix.length == 0) return null;
        double[] vector = matrix[0]; // 첫 행 사용
        float[] floatVector = new float[vector.length];
        for (int i = 0; i < vector.length; i++) {
            floatVector[i] = (float) vector[i];
        }
        return floatVector;
    }

    private <T> void addData(List<T> dataList, T data, String tag) {
        int maxSize = MAX_SIZE_DEFAULT;
        if (tag.contains("IMU")) {
            maxSize = MAX_SIZE_IMU;
        }
        if (dataList.size() >= maxSize) {
            dataList.remove(0);
            Log.d(tag, "데이터 제거");
        }
        dataList.add(data);
    }
}
