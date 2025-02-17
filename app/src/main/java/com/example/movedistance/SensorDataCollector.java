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
import android.telephony.CellSignalStrengthLte;
import android.telephony.TelephonyManager;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationServices;

import org.pytorch.Tensor;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SensorDataCollector extends AppCompatActivity {
    private static final int PROCESS_INTERVAL_60 = 60000; // 60초 간격
    private static final int PROCESS_INTERVAL_05 = 5000; // 5초 간격
    private static final int PROCESS_INTERVAL_01 = 1000; // 1초 간격
    private static final int MAX_SIZE = 60; // 최대 1분(60개) 저장
    private static final int PERMISSION_REQUEST_CODE = 1; //작동 횟수

    private final Handler handler = new Handler(Looper.getMainLooper());

    //AP == WIFI
    private final List<Map<String, Object>> apDataList = new ArrayList<>(); // // 원본 AP 데이터
    private final List<Map<String, Object>> apProcessedDataList = new ArrayList<>(); // 60초 간격 처리된 AP 데이터

    //BTS
    private final List<Map<String, Object>> btsDataList = new ArrayList<>(); // // 원본 BTS 데이터
    private final List<Map<String, Object>> btsProcessedDataList = new ArrayList<>(); // 5초 간격 처리된 BTS 데이터

    //GPS
    private final List<Map<String, Object>> gpsDataList = new ArrayList<>(); // 원본 GPS 데이터
    private final List<Map<String, Object>> gpsProcessedDataList = new ArrayList<>(); // 5초 간격 처리된 GPS 데이터

    //IMU
    private final List<Map<String, Object>> imuDataList = new ArrayList<>(); // 원본 IMU 데이터
    private final List<Map<String, Object>> imuProcessedDataList = new ArrayList<>(); // 1초 간격 처리된 IMU 데이터

    private WifiManager wifiManager;
    private TelephonyManager telephonyManager;
    private FusedLocationProviderClient fusedLocationProviderClient;
    private SensorManager sensorManager;
    private PyTorchHelper pyTorchHelper;
    private final ExecutorService executorService = Executors.newSingleThreadExecutor();
    private boolean notstartAI = true;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_ai);

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
    //권한 목록
    private boolean hasPermissions() {
        return ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED &&
                ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_WIFI_STATE) == PackageManager.PERMISSION_GRANTED &&
                ContextCompat.checkSelfPermission(this, Manifest.permission.READ_PHONE_STATE) == PackageManager.PERMISSION_GRANTED;
    }
    //권한 요청
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
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
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                long timestamp = System.currentTimeMillis();
                collectAPData(timestamp);
                collectBTSData(timestamp);
                collectGPSData(timestamp);
                collectIMUData(timestamp);
//                processIMUData();
                handler.postDelayed(this, PROCESS_INTERVAL_01); // 1초마다 반복 실행
            }
        }, PROCESS_INTERVAL_01);

        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                processAPData();
                processBTSData();
                processGPSData();
                processIMUData();
                processAI();
                handler.postDelayed(this, PROCESS_INTERVAL_60); // 60초마다 반복 실행
            }
        }, PROCESS_INTERVAL_60);

//        handler.postDelayed(new Runnable() {
//            @Override
//            public void run() {
//                processBTSData();
//                processGPSData();
//                processAI();
//                handler.postDelayed(this, PROCESS_INTERVAL_05); // 5초마다 반복 실행
//            }
//        }, PROCESS_INTERVAL_05);
    }
    //AI 연결
    private void processAI() {
        executorService.execute(() -> {
            if (pyTorchHelper == null) {
                Log.e("IMU", "PyTorchHelper 초기화");
                pyTorchHelper = new PyTorchHelper(this);
            }

            if (notstartAI) {
                notstartAI = false;
            } else {
                // ✅ 1. 모델이 요구하는 입력 크기 (340, 60)
                int sequenceLength = 340;  // 모델이 기대하는 시퀀스 길이
                int featureSize = 60;       // 모델이 기대하는 feature 개수

                float[] inputData = new float[sequenceLength * featureSize];

                // ✅ 2. 입력 데이터를 랜덤 값으로 채우기 (테스트용)
                for (int i = 0; i < inputData.length; i++) {
                    inputData[i] = (float) Math.random();
                }

                // ✅ 3. 입력 데이터 차원 맞추기 (1, 340, 60) → 3D 텐서로 변환
                Tensor inputTensor = Tensor.fromBlob(inputData, new long[]{1, sequenceLength, featureSize});

                // ✅ 4. PyTorch 모델 예측 실행
                float[] outputData = pyTorchHelper.predict(inputTensor);

                // ✅ 5. 예측 결과 출력
                Log.d("PyTorch Output", "Result: " + Arrays.toString(outputData));
            }
        });
    }


    //AP 데이터 처리
    private void processAPData() {
        if (!apDataList.isEmpty()) {
            long startTimestamp = System.currentTimeMillis() - (60 * 1000); // 최근 1분 데이터 기준
            List<Map<String, Object>> processedData = APProcessor.processAP(apDataList, startTimestamp);

            for (Map<String, Object> entry : processedData) {
                addData(apProcessedDataList, entry, "WiFi Processed");
            }
        }
    }

    private void collectAPData(long timestamp) {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_WIFI_STATE) == PackageManager.PERMISSION_GRANTED) {
            try {
                if (wifiManager != null) {
                    List<ScanResult> scanResults = wifiManager.getScanResults();
                    if (!scanResults.isEmpty()) {
                        ScanResult scanResult = scanResults.get(0);

                        // ✅ getCurrentSSID() 없이 SSID 가져오기
                        String ssid = "UNKNOWN_SSID";
                        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
                            WifiInfo wifiInfo = wifiManager.getConnectionInfo(); //문제는 없지만 나중에 수정 필요
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
                    }
                }
            } catch (SecurityException e) {
                Log.e("WiFi", "Wi-Fi 데이터 수집 실패: 권한이 부족함", e);
            }
        }
    }

    //BTS 데이터 처리
    private void processBTSData() {
        if (!btsDataList.isEmpty()) {
            long startTimestamp = System.currentTimeMillis() - (60 * 1000); // 최근 1분 데이터 기준
            List<Map<String, Object>> processedData = BTSProcessor.processBTS(btsDataList, startTimestamp);

            for (Map<String, Object> entry : processedData) {
                addData(btsProcessedDataList, entry, "BTS Processed");
            }
        }
    }
    private void collectBTSData(long timestamp) {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.READ_PHONE_STATE) == PackageManager.PERMISSION_GRANTED) {
            try {
                if (telephonyManager != null) {
                    List<CellInfo> cellInfoList = telephonyManager.getAllCellInfo();
                    for (CellInfo cellInfo : cellInfoList) {
                        if (cellInfo instanceof CellInfoLte) {
                            CellIdentityLte cellIdentity = ((CellInfoLte) cellInfo).getCellIdentity();
                            CellSignalStrengthLte signalStrength = ((CellInfoLte) cellInfo).getCellSignalStrength();

                            // ✅ formatBTSRawData() 없이 직접 데이터 생성
                            Map<String, Object> data = new HashMap<>();
                            data.put("timestamp", timestamp);
                            data.put("ci", cellIdentity.getCi());
                            data.put("pci", cellIdentity.getPci());

                            addData(btsDataList, data, "BTS Raw");
                        }
                    }
                }
            } catch (SecurityException e) {
                Log.e("BTS", "BTS 데이터 수집 실패: 권한이 부족함", e);
            }
        }
    }

    //GPS 데이터 처리
    private void processGPSData() {
        if (!gpsDataList.isEmpty()) {
            long startTimestamp = System.currentTimeMillis() - (60 * 1000); // 최근 1분 데이터 기준
            List<Map<String, Object>> processedData = GPSProcessor.processGPS(gpsDataList, startTimestamp);

            for (Map<String, Object> entry : processedData) {
                addData(gpsProcessedDataList, entry, "GPS Processed");
            }
        }
    }

    private void collectGPSData(long timestamp) {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            fusedLocationProviderClient.getLastLocation().addOnSuccessListener(location -> {
                if (location != null) {
                    Map<String, Object> data = Map.of(
                            "timestamp", timestamp,
                            "latitude", location.getLatitude(),
                            "longitude", location.getLongitude()
                    );
                    addData(gpsDataList, data, "GPS Raw");
                }
            });
        } else {
            Log.e("GPS", "GPS permission is missing.");
        }
    }

    //IMU 데이터 처리
    private void processIMUData() {
        if (!imuDataList.isEmpty()) {
            long startTimestamp = System.currentTimeMillis() - (60 * 1000); // 최근 1분 데이터 기준
            Map<String, double[][]> processedData = IMUProcessor.processIMU(imuDataList);

            for (Map.Entry<String, double[][]> entry : processedData.entrySet()) {
                Map<String, Object> dataMap = new HashMap<>();
                dataMap.put(entry.getKey(), entry.getValue()); // double[][] 데이터를 Object로 저장

                addData(imuProcessedDataList, dataMap, "IMU Processed");
            }
        }
    }
    private void collectIMUData(long timestamp) {
        if (sensorManager == null) {
            Log.e("IMU", "SensorManager is not initialized.");
            return;
        }

        // ✅ 센서 가져오기
        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor rotationVector = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        Sensor pressureSensor = sensorManager.getDefaultSensor(Sensor.TYPE_PRESSURE);
        Sensor gravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        Sensor linearAccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        // 하나라도 null이면 측정 불가
        if (accelerometer == null || gyroscope == null || magnetometer == null ||
                rotationVector == null || pressureSensor == null ||
                gravitySensor == null || linearAccelSensor == null) {
            Log.e("IMU", "One or more sensors are not available on this device.");
            return;
        }

        // ✅ 단발성 측정을 위한 배열/플래그
        //   - 각각의 센서 데이터를 저장할 공간
        final float[] accelValues = new float[3];
        final float[] gyroValues = new float[3];
        final float[] magValues = new float[3];
        final float[] rotValues = new float[4]; // 회전 벡터 (쿼터니언)
        final float[] gravityValues = new float[3];
        final float[] linearAccelValues = new float[3];
        final float[] pressureValue = new float[1]; // 기압은 float[1]로 처리

        // 각각 센서가 데이터를 한 번씩 받았는지 체크하기 위한 플래그
        final boolean[] gotSensorData = new boolean[7];
        // 인덱스: 0=accel, 1=gyro, 2=mag, 3=rot, 4=pressure, 5=gravity, 6=linear

        SensorEventListener oneTimeListener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                switch (event.sensor.getType()) {
                    case Sensor.TYPE_ACCELEROMETER:
                        System.arraycopy(event.values, 0, accelValues, 0, 3);
                        gotSensorData[0] = true;
                        break;

                    case Sensor.TYPE_GYROSCOPE:
                        System.arraycopy(event.values, 0, gyroValues, 0, 3);
                        gotSensorData[1] = true;
                        break;

                    case Sensor.TYPE_MAGNETIC_FIELD:
                        System.arraycopy(event.values, 0, magValues, 0, 3);
                        gotSensorData[2] = true;
                        break;

                    case Sensor.TYPE_ROTATION_VECTOR:
                        SensorManager.getQuaternionFromVector(rotValues, event.values);
                        gotSensorData[3] = true;
                        break;

                    case Sensor.TYPE_PRESSURE:
                        pressureValue[0] = event.values[0];
                        gotSensorData[4] = true;
                        break;

                    case Sensor.TYPE_GRAVITY:
                        System.arraycopy(event.values, 0, gravityValues, 0, 3);
                        gotSensorData[5] = true;
                        break;

                    case Sensor.TYPE_LINEAR_ACCELERATION:
                        System.arraycopy(event.values, 0, linearAccelValues, 0, 3);
                        gotSensorData[6] = true;
                        break;
                }

                // ✅ 모든 센서에서 한 번씩 데이터를 받았다면, 리스너 해제 후 한 번만 저장
                if (allSensorsReceived()) {
                    // 센서 데이터 저장
                    Map<String, Object> imuData = new HashMap<>();
                    imuData.put("timestamp", timestamp);
                    imuData.put("accel.x", accelValues[0]);
                    imuData.put("accel.y", accelValues[1]);
                    imuData.put("accel.z", accelValues[2]);
                    imuData.put("gyro.x", gyroValues[0]);
                    imuData.put("gyro.y", gyroValues[1]);
                    imuData.put("gyro.z", gyroValues[2]);
                    imuData.put("mag.x", magValues[0]);
                    imuData.put("mag.y", magValues[1]);
                    imuData.put("mag.z", magValues[2]);
                    imuData.put("rot.w", rotValues[0]);
                    imuData.put("rot.x", rotValues[1]);
                    imuData.put("rot.y", rotValues[2]);
                    imuData.put("rot.z", rotValues[3]);
                    imuData.put("pressure.x", pressureValue[0]);
                    imuData.put("gravity.x", gravityValues[0]);
                    imuData.put("gravity.y", gravityValues[1]);
                    imuData.put("gravity.z", gravityValues[2]);
                    imuData.put("linear_accel.x", linearAccelValues[0]);
                    imuData.put("linear_accel.y", linearAccelValues[1]);
                    imuData.put("linear_accel.z", linearAccelValues[2]);

                    addData(imuDataList, imuData, "IMU (One-Time)");
//                    Log.d("IMU", "✅IMU 데이터 수집 완료!");

                    // ✅ 센서 리스너 해제 (더 이상 데이터 수신 안 함)
                    sensorManager.unregisterListener(this);
                }
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
                // 필요 시 처리
            }

            // 모든 센서 데이터를 1회 이상 받았는지 확인
            private boolean allSensorsReceived() {
                for (boolean got : gotSensorData) {
                    if (!got) return false;
                }
                return true;
            }
        };

        // ✅ 센서 리스너 등록 (SENSOR_DELAY_GAME 예: 50~100Hz)
        sensorManager.registerListener(oneTimeListener, accelerometer, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(oneTimeListener, gyroscope, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(oneTimeListener, magnetometer, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(oneTimeListener, rotationVector, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(oneTimeListener, pressureSensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(oneTimeListener, gravitySensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(oneTimeListener, linearAccelSensor, SensorManager.SENSOR_DELAY_GAME);

//        Log.d("IMU", "✅ 단발성 센서 리스너 등록 완료 (수집 후 자동 해제).");
    }

    // ✅ 제네릭(Generic) 방식으로 모든 타입의 데이터를 저장
    private <T> void addData(List<T> dataList, T data, String tag) {
        if (dataList.size() >= MAX_SIZE) {
            dataList.remove(0); // FIFO 방식으로 가장 오래된 데이터 삭제
            Log.d(tag, "데이터 제거");
        }
        dataList.add(data);
        //Log.d(tag, data.toString());
    }
}