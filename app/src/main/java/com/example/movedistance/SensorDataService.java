package com.example.movedistance;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.telephony.CellIdentityLte;
import android.telephony.CellInfo;
import android.telephony.CellInfoLte;
import android.telephony.TelephonyManager;
import android.util.Log;

import androidx.core.content.ContextCompat;

import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationServices;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class SensorDataService extends Service {
    private static final int PROCESS_INTERVAL_01 = 1000; // 1초 간격
    private static final int IMU_INTERVAL_MS = 10; // 10ms 간격 (1초에 최대 100번)
    private static final int MAX_IMU_PER_SECOND = 100; // 1초에 최대 100개
    private static final int INITIAL_DELAY_MS = 3000; // 최초 3초 지연
    private static final String TAG = "SensorDataService";

    private WifiManager wifiManager;
    private TelephonyManager telephonyManager;
    private FusedLocationProviderClient fusedLocationProviderClient;
    private SensorManager sensorManager;
    private final Handler handler = new Handler(Looper.getMainLooper());
    private final ExecutorService executorService = Executors.newSingleThreadExecutor();

    private final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd", Locale.getDefault());
    private String currentDate;

    @Override
    public void onCreate() {
        super.onCreate();
        currentDate = dateFormat.format(new Date());

        wifiManager = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        telephonyManager = (TelephonyManager) getSystemService(Context.TELEPHONY_SERVICE);
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        fusedLocationProviderClient = LocationServices.getFusedLocationProviderClient(this);

        // 3초 후에 데이터 수집 시작
        handler.postDelayed(this::startDataCollection, INITIAL_DELAY_MS);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        return START_STICKY;
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    private void startDataCollection() {
        Runnable dataCollectionRunnable = new Runnable() {
            @Override
            public void run() {
                long timestamp = System.currentTimeMillis();
                collectAPData(timestamp);
                collectBTSData(timestamp);
                collectGPSData(timestamp);
                collectIMUData(timestamp);
                handler.postDelayed(this, PROCESS_INTERVAL_01);
            }
        };
        handler.post(dataCollectionRunnable);
    }

    // AP 데이터 수집 및 저장
    private void collectAPData(long timestamp) {
        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_WIFI_STATE)
                == PackageManager.PERMISSION_GRANTED) {
            try {
                if (wifiManager != null) {
                    List<ScanResult> scanResults = wifiManager.getScanResults();
                    if (!scanResults.isEmpty()) {
                        ScanResult scanResult = scanResults.get(0);
                        String ssid = "UNKNOWN_SSID";
                        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION)
                                == PackageManager.PERMISSION_GRANTED) {
                            WifiInfo wifiInfo = wifiManager.getConnectionInfo();
                            ssid = wifiInfo.getSSID();
                        }
                        Map<String, Object> data = new HashMap<>();
                        data.put("timestamp", timestamp);
                        data.put("bssid", scanResult.BSSID);
                        data.put("ssid", ssid);
                        data.put("level", scanResult.level);
                        data.put("frequency", scanResult.frequency);
                        data.put("capabilities", scanResult.capabilities);
                        saveToCSV("AP", data);
                    }
                }
            } catch (SecurityException e) {
                Log.e(TAG, "Wi-Fi 데이터 수집 실패: 권한 부족", e);
            }
        }
    }

    // BTS 데이터 수집 및 저장
    private void collectBTSData(long timestamp) {
        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.READ_PHONE_STATE)
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
                            saveToCSV("BTS", data);
                        }
                    }
                }
            } catch (SecurityException e) {
                Log.e(TAG, "BTS 데이터 수집 실패: 권한 부족", e);
            }
        }
    }

    // GPS 데이터 수집 및 저장
    private void collectGPSData(long timestamp) {
        if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED) {
            fusedLocationProviderClient.getLastLocation().addOnSuccessListener(location -> {
                if (location != null) {
                    Map<String, Object> data = new HashMap<>();
                    data.put("timestamp", timestamp);
                    data.put("latitude", location.getLatitude());
                    data.put("longitude", location.getLongitude());
                    saveToCSV("GPS", data);
                }
            });
        }
    }

    // IMU 데이터 수집 및 저장 (1초에 최대 100개)
    private void collectIMUData(long timestamp) {
        if (sensorManager == null) {
            Log.e(TAG, "SensorManager가 초기화되지 않음");
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
            Log.e(TAG, "필요한 센서가 장치에 없음");
            return;
        }

        final Map<Integer, float[]> sensorValues = new ConcurrentHashMap<>();
        final List<Map<String, Object>> imuDataBuffer = new ArrayList<>(MAX_IMU_PER_SECOND);
        final long startTime = System.currentTimeMillis();

        SensorEventListener listener = new SensorEventListener() {
            @Override
            public void onSensorChanged(SensorEvent event) {
                sensorValues.put(event.sensor.getType(), event.values.clone());
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {}
        };

        sensorManager.registerListener(listener, accelerometer, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(listener, gyroscope, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(listener, magnetometer, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(listener, rotationVector, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(listener, pressureSensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(listener, gravitySensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(listener, linearAccelSensor, SensorManager.SENSOR_DELAY_GAME);

        Runnable imuCollector = new Runnable() {
            int count = 0;

            @Override
            public void run() {
                if (count >= MAX_IMU_PER_SECOND || System.currentTimeMillis() - startTime >= 1000) {
                    sensorManager.unregisterListener(listener);
                    for (Map<String, Object> data : imuDataBuffer) {
                        saveToCSV("IMU", data);
                    }
                    return;
                }

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
                    data.put("pressure", pressure[0]);

                    float[] gravity = sensorValues.get(Sensor.TYPE_GRAVITY);
                    data.put("gravity.x", gravity[0]);
                    data.put("gravity.y", gravity[1]);
                    data.put("gravity.z", gravity[2]);

                    float[] linear = sensorValues.get(Sensor.TYPE_LINEAR_ACCELERATION);
                    data.put("linear_accel.x", linear[0]);
                    data.put("linear_accel.y", linear[1]);
                    data.put("linear_accel.z", linear[2]);

                    imuDataBuffer.add(data);
                    count++;
                }
                handler.postDelayed(this, IMU_INTERVAL_MS);
            }
        };

        handler.post(imuCollector);
    }

    // CSV 파일로 데이터 저장
    private void saveToCSV(String sensorType, Map<String, Object> data) {
        executorService.execute(() -> {
            try {
                File directory = new File(getExternalFilesDir(null), "SensorData");
                if (!directory.exists()) {
                    directory.mkdirs();
                }

                String fileName = currentDate + "_" + sensorType + ".csv";
                File file = new File(directory, fileName);
                boolean isNewFile = !file.exists();

                try (FileWriter writer = new FileWriter(file, true)) {
                    if (isNewFile) {
                        writer.append(String.join(",", data.keySet())).append("\n");
                    }
                    StringBuilder line = new StringBuilder();
                    for (Object value : data.values()) {
                        if (line.length() > 0) line.append(",");
                        line.append(value.toString());
                    }
                    writer.append(line.toString()).append("\n");
                }
            } catch (IOException e) {
                Log.e(TAG, "CSV 저장 실패: " + sensorType, e);
            }
        });
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        handler.removeCallbacksAndMessages(null);
        executorService.shutdown();
        if (sensorManager != null) {
            sensorManager.unregisterListener((SensorEventListener) this);
        }
    }
}