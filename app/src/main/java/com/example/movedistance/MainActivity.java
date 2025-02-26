package com.example.movedistance;

import androidx.appcompat.app.AppCompatActivity;
import android.content.Intent;
import android.os.Bundle;;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Intent serviceIntent = new Intent(this, SensorDataService.class);
        //모델 태스트용
        //PyTorchHelper.runPrediction(this);
        getSupportFragmentManager().beginTransaction()
                .replace(R.id.fragment_container, new MapFragment())
                .commit();

        startService(serviceIntent);
        SensorDataProcessor.scheduleBackgroundPrediction(this);
    }
}