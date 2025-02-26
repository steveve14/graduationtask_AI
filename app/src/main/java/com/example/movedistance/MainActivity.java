package com.example.movedistance;

import androidx.appcompat.app.AppCompatActivity;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.Button;
import android.widget.TextView;

import org.osmdroid.views.MapView;

import com.example.movedistance.MAP.GpsTracker;
import com.example.movedistance.MAP.OfflineMapManager;

import java.util.HashMap;
import java.util.Map;

public class MainActivity extends AppCompatActivity {
    private static final float NANO_TO_SECONDS = 1_000_000_000.0f;
    private static final Map<String, Float> EMISSION_RATES = new HashMap<>();
    private static final float[] SPEED_THRESHOLDS = {0.0033f, 0.0083f, 0.0250f, 0.0361f};
    private static final String[] TRANSPORT_MODES = {"걷기", "자전거", "버스", "자동차", "지하철"};

    static {
        EMISSION_RATES.put("버스", 100f);
        EMISSION_RATES.put("자동차", 200f);
        EMISSION_RATES.put("지하철", 50f);
    }

    private TextView textDistance, textStatus, textAI;
    private Button btnToggle, btnAI;
    private SensorManager sensorManager;
    private Sensor accelerometer;
    private final SensorListener sensorListener = new SensorListener();

    private float totalDistance = 0;
    private long lastUpdateTime = 0;
    private float carbonEmissions = 0;
    private boolean isMeasuring = false;
    private final Map<String, Float> transportDistances = new HashMap<>();

    private OfflineMapManager offlineMapManager;
    private GpsTracker gpsTracker;
    private MapView mapView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Intent serviceIntent = new Intent(this, SensorDataService.class);
        //PyTorchHelper.runPrediction(this);
        startService(serviceIntent);
        SensorDataProcessor.scheduleBackgroundPrediction(this);

        initUI();
        initSensors();
    }

    private void initUI() {
        textDistance = findViewById(R.id.textView2);
        textStatus = findViewById(R.id.ing);
        textAI = findViewById(R.id.textViewAI);
        btnToggle = findViewById(R.id.start);
        btnAI = findViewById(R.id.startai);

        btnToggle.setOnClickListener(v -> toggleMeasurement());
        btnAI.setOnClickListener(v -> startAIActivity());
    }

    private void initSensors() {
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    }

    private void startAIActivity() {
        Intent intent = new Intent(this, SensorDataCollector.class);
        startActivity(intent);
        overridePendingTransition(android.R.anim.fade_in, android.R.anim.fade_out);
    }

    private class SensorListener implements SensorEventListener {
        private float lastAcceleration = 0;
        private final StringBuilder displayBuilder = new StringBuilder(128);

        @Override
        public void onSensorChanged(SensorEvent event) {
            if (!isMeasuring || event.sensor.getType() != Sensor.TYPE_ACCELEROMETER) return;

            float accX = event.values[0];
            float accY = event.values[1];
            float accZ = event.values[2];

            long now = System.nanoTime();
            float deltaTime = (now - lastUpdateTime) / NANO_TO_SECONDS;
            lastUpdateTime = now;

            float acceleration = (float) Math.sqrt(accX * accX + accY * accY + accZ * accZ);
            float effectiveAcc = acceleration - lastAcceleration;
            float distance = Math.abs(effectiveAcc) * 0.5f * deltaTime * deltaTime;

            totalDistance += distance;
            lastAcceleration = acceleration;

            String transportMode = classifyTransportMode(distance, deltaTime);
            transportDistances.merge(transportMode, distance, Float::sum);
            carbonEmissions += calculateCarbonEmission(transportMode, distance);

            updateUI(transportMode);
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int accuracy) {}
    }

    private String classifyTransportMode(float distance, float deltaTime) {
        float speedKmS = (distance / deltaTime) * 0.001f;
        for (int i = 0; i < SPEED_THRESHOLDS.length; i++) {
            if (speedKmS < SPEED_THRESHOLDS[i]) return TRANSPORT_MODES[i];
        }
        return TRANSPORT_MODES[TRANSPORT_MODES.length - 1];
    }

    private float calculateCarbonEmission(String mode, float distance) {
        return (distance / 1000) * EMISSION_RATES.getOrDefault(mode, 0f);
    }

    private void updateUI(String transportMode) {
        StringBuilder sb = sensorListener.displayBuilder;
        sb.setLength(0);
        sb.append(String.format("총 이동 거리: %.2f m\n", totalDistance))
                .append(String.format("예상 이동 수단: %s\n", transportMode))
                .append(String.format("총 탄소 배출량: %.2f gCO₂\n", carbonEmissions));

        for (Map.Entry<String, Float> entry : transportDistances.entrySet()) {
            sb.append(String.format("%s 이동 거리: %.2f m\n", entry.getKey(), entry.getValue()));
        }
        textDistance.setText(sb.toString());
    }

    private void toggleMeasurement() {
        isMeasuring = !isMeasuring;
        if (isMeasuring) {
            textStatus.setText("측정 중...");
            btnToggle.setText("측정 끝내기");
            resetData();
            lastUpdateTime = System.nanoTime();
            sensorManager.registerListener(sensorListener, accelerometer, SensorManager.SENSOR_DELAY_UI);
        } else {
            textStatus.setText("측정 완료");
            btnToggle.setText("다시 측정하기");
            sensorManager.unregisterListener(sensorListener);
        }
    }

    private void resetData() {
        totalDistance = 0;
        carbonEmissions = 0;
        transportDistances.clear();
        sensorListener.lastAcceleration = 0;
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        sensorManager.unregisterListener(sensorListener);
        if (gpsTracker != null) gpsTracker.stopTracking();
    }
}