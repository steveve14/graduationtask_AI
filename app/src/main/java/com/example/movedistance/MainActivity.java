package com.example.movedistance;

import androidx.appcompat.app.AppCompatActivity;
import android.annotation.SuppressLint;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.Button;
import android.widget.TextView;

import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.MapView;

import com.example.movedistance.MAP.GpsTracker;
import com.example.movedistance.MAP.OfflineMapManager;

import java.util.HashMap;
import java.util.Map;

public class MainActivity extends AppCompatActivity {
    private TextView text2, text5, textAI, ing;
    private Button startbtn, startai;
    private SensorManager manager;
    private SensorEventListener listener;

    private float totalDistance = 0;
    private long lastUpdateTime = 0;

    private final Map<String, Float> transportDistances = new HashMap<>();
    private float carbonEmissions = 0;
    private boolean firstRun = true;
    private boolean isMeasuring = false;

    private OfflineMapManager offlineMapManager;
    private GpsTracker gpsTracker;
    private MapView mapView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initUI();
        initSensors();
    }

    private void initUI() {
        text2 = findViewById(R.id.textView2);
        text5 = findViewById(R.id.textView5);
        textAI = findViewById(R.id.textViewAI);
        ing = findViewById(R.id.ing);

        startbtn = findViewById(R.id.start);
        startai = findViewById(R.id.startai);

        startbtn.setOnClickListener(v -> toggleMeasurement());
        startai.setOnClickListener(v -> {
            Intent intent = new Intent(MainActivity.this, SensorDataCollector.class);
            startActivity(intent);
            overridePendingTransition(android.R.anim.fade_in, android.R.anim.fade_out);
        });
    }

    private void initSensors() {

        manager = (SensorManager) getSystemService(SENSOR_SERVICE);
        Sensor accelerometer = manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        Sensor rotationSensor = manager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        Sensor pressureSensor = manager.getDefaultSensor(Sensor.TYPE_PRESSURE);

        listener = new SensorEventListener() {
            private float lastAcceleration = 0;

            @SuppressLint({"SetTextI18n", "DefaultLocale"})
            @Override
            public void onSensorChanged(SensorEvent event) {
                if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
                    float accX = event.values[0];
                    float accY = event.values[1];
                    float accZ = event.values[2];

                    long now = System.nanoTime();
                    float deltaTime = (now - lastUpdateTime) / 1_000_000_000.0f; // ns → s
                    lastUpdateTime = now;

                    float acceleration = (float) Math.sqrt(accX * accX + accY * accY + accZ * accZ);
                    float effectiveAcc = acceleration - lastAcceleration;
                    float distance = Math.abs(effectiveAcc) * 0.5f * deltaTime * deltaTime;
                    totalDistance += distance;
                    lastAcceleration = acceleration;

                    String transportMode = classifyTransportMode(distance, deltaTime);
                    updateUI(transportMode, distance);
                }
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
            }
        };
    }

    private String classifyTransportMode(float distance, float deltaTime) {
        float speed_km_s = (distance / deltaTime) * 0.001f;
        if (speed_km_s < 0.0033) return "걷기";
        if (speed_km_s < 0.0083) return "자전거";
        if (speed_km_s < 0.0250) return "버스";
        if (speed_km_s < 0.0361) return "자동차";
        return "지하철";
    }

    private float calculateCarbonEmission(String mode, float distance) {
        Map<String, Float> emissionRates = Map.of(
                "버스", 100f,
                "자동차", 200f,
                "지하철", 50f
        );
        return (distance / 1000) * emissionRates.getOrDefault(mode, 0f);
    }

    private void updateUI(String transportMode, float distance) {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("총 이동 거리: %.2f m\n", totalDistance));
        sb.append(String.format("예상 이동 수단: %s\n", transportMode));
        sb.append(String.format("총 탄소 배출량: %.2f gCO₂\n", carbonEmissions));

        for (Map.Entry<String, Float> entry : transportDistances.entrySet()) {
            sb.append(String.format("%s 이동 거리: %.2f m\n", entry.getKey(), entry.getValue()));
        }
        text2.setText(sb.toString());
    }

    private void toggleMeasurement() {
        if (isMeasuring) {
            ing.setText("측정 완료");
            startbtn.setText("다시 측정하기");
            manager.unregisterListener(listener);
        } else {
            ing.setText("측정 중...");
            startbtn.setText("측정 끝내기");
            resetData();
            manager.registerListener(listener, manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_UI);
        }
        isMeasuring = !isMeasuring;
    }

    private void resetData() {
        totalDistance = 0;
        carbonEmissions = 0;
        transportDistances.clear();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        gpsTracker.stopTracking();
    }
}
