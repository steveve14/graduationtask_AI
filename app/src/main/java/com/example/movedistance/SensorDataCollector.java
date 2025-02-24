package com.example.movedistance;

import android.Manifest;
import android.annotation.SuppressLint;
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
import java.util.Collections;
import java.util.Comparator;
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
    private static final int MAX_SIZE_IMU = 6000;

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
        processAI();
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
        // 데이터 수집을 위한 Runnable 정의
        Runnable dataCollectionRunnable = new Runnable() {
            @Override
            public void run() {
                long timestamp = System.currentTimeMillis();
                collectAPData(timestamp);
                collectBTSData(timestamp);
                collectGPSData(timestamp);
                collectIMUData(timestamp);
                handler.postDelayed(this, PROCESS_INTERVAL_01); // 지속적인 데이터 수집을 위해 다시 예약
            }
        };
        // 데이터 처리를 위한 Runnable 정의
        Runnable dataProcessingRunnable = new Runnable() {
            @Override
            public void run() {
                processAPData();
                processBTSData();
                processGPSData();
                processIMUData();

                handler.postDelayed(this, PROCESS_INTERVAL_60); // 지속적인 데이터 처리를 위해 다시 예약
            }
        };
        // Runnable 예약
        handler.postDelayed(dataCollectionRunnable, PROCESS_INTERVAL_01);
        handler.postDelayed(dataProcessingRunnable, PROCESS_INTERVAL_60);
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
                        data.put("bssid", scanResult.BSSID);
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


    // AP 데이터 처리
    private void processAPData() {
        if (!apDataList.isEmpty()) {
            List<Map<String, Object>> processedData = APProcessor.processAP(apDataList, findEarliestTimestamp(apDataList));
            StringBuilder dataText = new StringBuilder("Processed AP Data:\n");
            for (Map<String, Object> entry : processedData) {
                addData(apProcessedDataList, entry, "WiFi Processed");
                dataText.append(entry.toString()).append("\n");
            }
            runOnUiThread(() -> textAP.setText(dataText.toString()));

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
            List<Map<String, Object>> processedData = BTSProcessor.processBTS(btsDataList, findEarliestTimestamp(btsDataList));
            StringBuilder dataText = new StringBuilder("Processed BTS Data:\n");
            for (Map<String, Object> entry : processedData) {
                addData(btsProcessedDataList, entry, "BTS Processed");
                dataText.append(entry.toString()).append("\n");
            }
            runOnUiThread(() -> textBTS.setText(dataText.toString()));
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
                }
            });
        } else {
            Log.e("GPS", "GPS permission is missing.");
        }
    }

    // GPS 데이터 처리
    private void processGPSData() {
        if (!gpsDataList.isEmpty()) {
            List<Map<String, Object>> processedData = GPSProcessor.processGPS(gpsDataList, findEarliestTimestamp(gpsDataList));

            StringBuilder dataText = new StringBuilder("Processed GPS Data:\n");
            for (Map<String, Object> entry : processedData) {
                addData(gpsProcessedDataList, entry, "GPS Processed");
                dataText.append(entry.toString()).append("\n");
            }
            runOnUiThread(() -> textGPS.setText(dataText.toString()));
        }
    }
    // IMU 데이터 누적 수집
    private void collectIMUData(long timestamp) {
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

        // 10ms 간격으로 센서 데이터를 기록
        handler.postDelayed(new Runnable() {
            @SuppressLint("SetTextI18n")
            @Override
            public void run() {
                if (System.currentTimeMillis() - timestamp >= 1000) { // 1초(1000ms) 경과 후 종료
                    sensorManager.unregisterListener(accumulationListener);
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
                    data.put("timestamp", timestamp);

                    float[] accel = sensorValues.get(Sensor.TYPE_ACCELEROMETER);
                    assert accel != null;
                    data.put("accel.x", accel[0]);
                    data.put("accel.y", accel[1]);
                    data.put("accel.z", accel[2]);

                    float[] gyro = sensorValues.get(Sensor.TYPE_GYROSCOPE);
                    assert gyro != null;
                    data.put("gyro.x", gyro[0]);
                    data.put("gyro.y", gyro[1]);
                    data.put("gyro.z", gyro[2]);

                    float[] mag = sensorValues.get(Sensor.TYPE_MAGNETIC_FIELD);
                    assert mag != null;
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
                    assert pressure != null;
                    data.put("pressure.x", pressure[0]);

                    float[] gravity = sensorValues.get(Sensor.TYPE_GRAVITY);
                    assert gravity != null;
                    data.put("gravity.x", gravity[0]);
                    data.put("gravity.y", gravity[1]);
                    data.put("gravity.z", gravity[2]);

                    float[] linear = sensorValues.get(Sensor.TYPE_LINEAR_ACCELERATION);
                    assert linear != null;
                    data.put("linear_accel.x", linear[0]);
                    data.put("linear_accel.y", linear[1]);
                    data.put("linear_accel.z", linear[2]);

                    addData(imuDataList, data, "IMU Accumulated");
                }
                handler.postDelayed(this, 10);  // 10ms 간격으로 실행 (총 100회 반복)
            }
        }, 5);
    }

    // IMU 데이터 처리 (누적 데이터 기반)
    private void processIMUData() {
        if(!imuDataList.isEmpty()){
            findEarliestTimestamp(imuDataList);
            List<Map<String, Object>> processedData = IMUProcessor.preImu(imuDataList);

            StringBuilder dataText = new StringBuilder("Processed IMU Data:\n");
            for (Map<String, Object> entry : processedData) {
                addData(imuProcessedDataList, entry, "IMU Processed");
                dataText.append(entry.toString()).append("\n");
            }
            runOnUiThread(() -> textIMU.setText(dataText.toString()));
        }
        processAI();
    }

    private static long findEarliestTimestamp(List<Map<String, Object>> dataList) {
        return dataList.stream()
                .filter(map -> map.containsKey("timestamp"))
                .map(map -> (long) map.get("timestamp"))
                .min(Long::compare)
                .orElse(System.currentTimeMillis());
    }

    // AI 처리 - 여기서 AP, BTS, GPS, IMU 순서로 병합된 피처 벡터를 AI 모델의 입력으로 사용합니다.
    @SuppressLint("SetTextI18n")
    private void processAI() {
        executorService.execute(() -> {
            if (pyTorchHelper == null) {
                pyTorchHelper = new PyTorchHelper(this);
                runOnUiThread(() -> textAIResult.setText("PyTorchHelper 초기화 완료"));
            }

            if (notstartAI) {
                notstartAI = false;
            } else {
                float[][][] inputFeatureVector = getProcessedFeatureVector();
                if (inputFeatureVector == null) {
                    Log.e("AI", "입력 벡터 생성 실패");
                    return;
                }

                // 3D 데이터를 1D로 변환하여 Tensor에 입력
                float[] flattenedVector = new float[340 * 60];
                int index = 0;
                for (float[][] step : inputFeatureVector) {
                    for (float[] feature : step) {
                        System.arraycopy(feature, 0, flattenedVector, index, feature.length);
                        index += feature.length;
                    }
                }

                Tensor inputTensor = Tensor.fromBlob(flattenedVector, new long[]{1, 340, 60});
                float[] outputData = pyTorchHelper.predict(inputTensor);

                Log.d("PyTorch Output", "Result: " + Arrays.toString(outputData));
                runOnUiThread(() -> textAIResult.setText(String.valueOf(outputData[0])));
            }
        });
    }

    private float[][] extractFeatureVectorFromList(List<Map<String, Object>> list, int originalSize, int expansionFactor) {
        if (list.size() < originalSize) return null;

        float[][] expandedData = new float[originalSize * expansionFactor][];
        int index = 0;
        for (int i = 0; i < originalSize; i++) {
            float[] vector = extractFeatureVectorFromMap(list.get(i));
            if (vector == null) return null;
            for (int j = 0; j < expansionFactor; j++) {
                expandedData[index++] = vector;
            }
        }
        return expandedData;
    }

    private float[][][] getProcessedFeatureVector() {
        printsize("AP", apProcessedDataList);
        printsize("BTS", btsProcessedDataList);
        printsize("GPS", gpsProcessedDataList);
        printsize("IMU", imuProcessedDataList);

        if (apProcessedDataList.isEmpty()) {
            Log.e("AI", "AP 데이터가 없습니다.");
            return null;
        }
        if (btsProcessedDataList.isEmpty()) {
            Log.e("AI", "BTS 데이터가 없습니다.");
            return null;
        }
        if (gpsProcessedDataList.isEmpty()) {
            Log.e("AI", "GPS 데이터가 없습니다.");
            return null;
        }
        if (imuProcessedDataList.isEmpty()) {
            Log.e("AI", "IMU 데이터가 없습니다.");
            return null;
        }

        float[][][] inputVector = new float[1][340][60];

        // **AP 데이터 변환 (1개 → 60개로 복제)**
        Log.d("AI", "AP 데이터 리스트 크기: " + apProcessedDataList.size());
        List<Map<String, Object>> sortedAPDataList = sortAndRemoveTimestamp(apProcessedDataList);
        float[] apFeatures = extractFeatureVectorFromMap(sortedAPDataList.get(sortedAPDataList.size() - 1));
        if (apFeatures == null || apFeatures.length < 60) {
            Log.e("AI", "AP feature vector is null or too short");
            return null;
        }
        Log.d("AI", "AP feature vector 크기: " + apFeatures.length);
        for (int i = 0; i < Math.min(60, apFeatures.length); i++) {
            inputVector[0][0][i] = apFeatures[i];
        }

        // **BTS 데이터 변환 (12개 → 60개, 1개당 5개 복제)**
        List<Map<String, Object>> sortedBTSDataList = sortAndRemoveTimestamp(btsProcessedDataList);
        float[][] btsFeatures = extractFeatureVectorFromList(sortedBTSDataList, 12, 5);
        if (btsFeatures == null || btsFeatures.length < 5) {
            Log.e("AI", "BTS feature vector is null or too short");
            return null;
        }
        Log.d("AI", "BTS feature vector 크기: " + btsFeatures.length + " x " + (btsFeatures.length > 0 ? btsFeatures[0].length : 0));
        for (int col = 1; col <= 5; col++) {
            System.arraycopy(btsFeatures[col - 1], 0, inputVector[0][col], 0, Math.min(60, btsFeatures[col - 1].length));
        }

        // **GPS 데이터 변환 (12개 → 60개, 1개당 5개 복제)**
        Log.d("AI", "GPS 데이터 리스트 크기: " + gpsProcessedDataList.size());
        List<Map<String, Object>> sortedGPSDataList = sortAndRemoveTimestamp(gpsProcessedDataList);
        float[][] gpsFeatures = extractFeatureVectorFromList(sortedGPSDataList, 12, 5);
        if (gpsFeatures == null || gpsFeatures.length < 4) {
            Log.e("AI", "GPS feature vector is null or too short");
            return null;
        }
        Log.d("AI", "GPS feature vector 크기: " + gpsFeatures.length + " x " + (gpsFeatures.length > 0 ? gpsFeatures[0].length : 0));
        for (int col = 6; col <= 9; col++) {
            System.arraycopy(gpsFeatures[col - 6], 0, inputVector[0][col], 0, Math.min(60, gpsFeatures[col - 6].length));
        }

        // **IMU 데이터 변환 (60개 → 60개, 1대1 매칭)**
        Log.d("AI", "IMU 데이터 리스트 크기: " + imuProcessedDataList.size());
        List<Map<String, Object>> sortedIMUDataList = sortAndRemoveTimestamp(imuProcessedDataList);
        float[][] imuFeatures = extractFeatureVectorFromList(adjustImuDataList(sortedIMUDataList, 60), 60, 1);
        if (imuFeatures == null || imuFeatures.length < 330) {
            Log.e("AI", "IMU feature vector is null or too short");
            return null;
        }
        Log.d("AI", "IMU feature vector 크기: " + imuFeatures.length + " x " + (imuFeatures.length > 0 ? imuFeatures[0].length : 0));
        for (int col = 10; col <= 339; col++) {
            System.arraycopy(imuFeatures[col - 10], 0, inputVector[0][col], 0, Math.min(60, imuFeatures[col - 10].length));
        }

        // 최종 입력 벡터 크기 확인
        Log.d("AI", "최종 입력 벡터 크기: " + inputVector.length + " x " + inputVector[0].length + " x " + inputVector[0][0].length);

        return inputVector;
    }

    private void printsize( String tag, List<Map<String, Object>> data){
        if (!data.isEmpty()) {
            int numberOfColumns = data.get(0).size();
            Log.d(tag,"열의 개수:"+numberOfColumns);
        } else {
            Log.d(tag,"리스트가 비어 있습니다.");
        }
    }

    private List<Map<String, Object>> sortAndRemoveTimestamp(List<Map<String, Object>> dataList) {
        Collections.sort(dataList, new Comparator<Map<String, Object>>() {
            @Override
            public int compare(Map<String, Object> o1, Map<String, Object> o2) {
                Long timestamp1 = (Long) o1.get("timestamp");
                Long timestamp2 = (Long) o2.get("timestamp");

                if (timestamp1 == null || timestamp2 == null) {
                    Log.e("Sorting", "One or both timestamps are null");
                    return 0; // Consider equal if either timestamp is null
                }

                return timestamp1.compareTo(timestamp2);
            }
        });

        // Remove the timestamp from each map
        for (Map<String, Object> data : dataList) {
            data.remove("timestamp");
        }

        return dataList;
    }

    // 맵에서 피처 벡터 추출
    private float[] extractFeatureVectorFromMap(Map<String, Object> map) {

        if (map == null || map.isEmpty()) {
            Log.e("AI", "Map is null or empty");
            return null;
        }

        Object value = map.values().iterator().next();

        if (value == null) {
            Log.e("AI", "Feature value is null");
            return null;
        }

        //Log.d("AI", "Feature value class: " + value.getClass().getName());

        double[][] matrix = null;

        // 자동 변환 처리
        if (value instanceof double[][]) {
            matrix = (double[][]) value;
        } else if (value instanceof double[]) {
            matrix = new double[][]{(double[]) value}; // 1차원 배열을 2차원 배열로 변환
        } else if (value instanceof List) {
            List<?> list = (List<?>) value;

            if (!list.isEmpty()) {
                if (list.get(0) instanceof List) {
                    // List<List<Double>> → double[][]
                    List<List<Double>> nestedList = (List<List<Double>>) list;
                    matrix = new double[nestedList.size()][];
                    for (int i = 0; i < nestedList.size(); i++) {
                        matrix[i] = nestedList.get(i).stream().mapToDouble(Double::doubleValue).toArray();
                    }
                } else if (list.get(0) instanceof Double) {
                    // List<Double> → double[][]
                    List<Double> singleList = (List<Double>) list;
                    matrix = new double[][]{singleList.stream().mapToDouble(Double::doubleValue).toArray()};
                }
            }
        } else if (value instanceof Long || value instanceof Integer || value instanceof Double) {
            // 단일 숫자값(Long, Integer, Double) → double[][]
            matrix = new double[][]{{((Number) value).doubleValue()}};
        } else {
            Log.e("AI", "Unsupported data type: " + value.getClass().getName());
            return null;
        }

        if (matrix == null || matrix.length == 0) {
            Log.e("AI", "Matrix is empty or null after conversion");
            return null;
        }

        double[] vector = matrix[0]; // 첫 행 사용
        float[] floatVector = new float[vector.length];

        for (int i = 0; i < vector.length; i++) {
            floatVector[i] = (float) vector[i];
        }

        return floatVector;
    }

    private List<Map<String, Object>> adjustImuDataList(List<Map<String, Object>> imuDataList, int targetSize) {
        int originalSize = imuDataList.size();

        if (originalSize == targetSize) {
            // 이미 원하는 크기인 경우
            return imuDataList;
        }

        // 새로운 리스트 생성
        List<Map<String, Object>> adjustedList = new ArrayList<>(imuDataList);

        // 60개보다 작다면 마지막 값을 복제하여 채운다.
        if (originalSize < targetSize) {
            Map<String, Object> lastValue = imuDataList.get(originalSize - 1);
            for (int i = originalSize; i < targetSize; i++) {
                // 마지막 값을 복제하여 추가
                adjustedList.add(new HashMap<>(lastValue));
            }
        } else if (originalSize > targetSize) {
            // 60개보다 크다면 초과분을 잘라낸다.
            adjustedList = adjustedList.subList(0, targetSize);
        }

        return adjustedList;
    }


    private <T> void addData(List<T> dataList, T data, String tag) {
        int maxSize = MAX_SIZE_DEFAULT;
        if (tag.contains("IMU")) {
            maxSize = MAX_SIZE_IMU;
        }
        if (dataList.size() >= maxSize) {
            dataList.remove(0);
//            Log.d(tag, "데이터 제거");
        }
        dataList.add(data);
    }
}
