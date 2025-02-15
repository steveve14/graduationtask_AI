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

import java.util.ArrayList;
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

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        wifiManager = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        telephonyManager = (TelephonyManager) getSystemService(Context.TELEPHONY_SERVICE);
        SensorManager sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
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

                processIMUData();
                handler.postDelayed(this, PROCESS_INTERVAL_01); // 1초마다 반복 실행
            }
        }, PROCESS_INTERVAL_01);

        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                processAPData();
                handler.postDelayed(this, PROCESS_INTERVAL_60); // 60초마다 반복 실행
            }
        }, PROCESS_INTERVAL_60);

        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                processBTSData();
                processGPSData();
                handler.postDelayed(this, PROCESS_INTERVAL_05); // 5초마다 반복 실행
            }
        }, PROCESS_INTERVAL_05);


    }

    //AP 데이터 처리
    private void processAPData() {
        if (!apDataList.isEmpty()) {
            long startTimestamp = System.currentTimeMillis() - (60 * 1000); // 최근 1분 데이터 기준
            List<Map<String, Object>> processedData = APProcessor.processAP(apDataList, startTimestamp);

            for (Map<String, Object> entry : processedData) {
                addData(apProcessedDataList, entry, "WiFi Processed");
            }

            // ✅ BTS 처리 방식과 동일하게 1분 이전 데이터 삭제
            apDataList.removeIf(record -> (long) record.get("timestamp") < startTimestamp);
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

            // ✅ 원본 데이터 리스트 정리 (1분 이전 데이터 삭제)
            btsDataList.removeIf(record -> (long) record.get("timestamp") < startTimestamp);
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

            // ✅ 1분 이전 데이터 삭제
            gpsDataList.removeIf(record -> (long) record.get("timestamp") < startTimestamp);
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

            // ✅ 1분 이전 데이터 삭제
            imuDataList.removeIf(record -> (long) record.get("timestamp") < startTimestamp);
        }
    }
    private void collectIMUData(long timestamp) {
        if (sensorManager == null) {
            Log.e("IMU", "SensorManager is not initialized.");
            return;
        }

        Sensor accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        Sensor magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        if (accelerometer == null || gyroscope == null || magnetometer == null) {
            Log.e("IMU", "One or more sensors are not available on this device.");
            return;
        }

        SensorEventListener sensorEventListener = new SensorEventListener() {
            private final float[] accelValues = new float[3];
            private final float[] gyroValues = new float[3];
            private final float[] magValues = new float[3];

            @Override
            public void onSensorChanged(SensorEvent event) {
                if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                    System.arraycopy(event.values, 0, accelValues, 0, event.values.length);
                } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
                    System.arraycopy(event.values, 0, gyroValues, 0, event.values.length);
                } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                    System.arraycopy(event.values, 0, magValues, 0, event.values.length);
                }

                // ✅ 모든 센서 데이터가 업데이트되면 저장
                Map<String, Object> imuData = Map.of(
                        "timestamp", timestamp,
                        "accel_x", accelValues[0], "accel_y", accelValues[1], "accel_z", accelValues[2],
                        "gyro_x", gyroValues[0], "gyro_y", gyroValues[1], "gyro_z", gyroValues[2],
                        "mag_x", magValues[0], "mag_y", magValues[1], "mag_z", magValues[2]
                );

                addData(imuDataList, imuData, "IMU");
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
                // 정확도 변경 시 처리 필요하면 추가
            }
        };

        // ✅ SENSOR_DELAY_GAME 설정 (50~100Hz)
        sensorManager.registerListener(sensorEventListener, accelerometer, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(sensorEventListener, gyroscope, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(sensorEventListener, magnetometer, SensorManager.SENSOR_DELAY_GAME);
    }

    // ✅ 제네릭(Generic) 방식으로 모든 타입의 데이터를 저장
    private <T> void addData(List<T> dataList, T data, String tag) {
        if (dataList.size() >= MAX_SIZE) {
            dataList.remove(0); // FIFO 방식으로 가장 오래된 데이터 삭제
        }
        dataList.add(data);
        Log.d(tag, data.toString());
    }
}