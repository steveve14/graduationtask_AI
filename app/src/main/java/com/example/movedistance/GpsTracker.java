package com.example.movedistance;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.location.Location;
import androidx.core.app.ActivityCompat;
import com.google.android.gms.location.*;

public class GpsTracker {
    private final Context context;
    private final FusedLocationProviderClient fusedLocationClient;
    private LocationCallback locationCallback;
    private Location previousLocation;
    private long previousTime;
    private LocationUpdateCallback locationUpdateCallback; // 콜백 인터페이스 대신 직접 설정

    public interface LocationUpdateCallback {
        void onLocationUpdated(Location location, float speed);
    }

    public GpsTracker(Context context) {
        this.context = context;
        fusedLocationClient = LocationServices.getFusedLocationProviderClient(context);
        initializeLocationUpdates();
    }

    private void initializeLocationUpdates() {
        LocationRequest locationRequest = LocationRequest.create()
                .setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY)
                .setInterval(5000); // 5초마다 업데이트

        locationCallback = new LocationCallback() {
            @Override
            public void onLocationResult(LocationResult locationResult) {
                if (locationResult == null) return;

                for (Location location : locationResult.getLocations()) {
                    long currentTime = System.currentTimeMillis();
                    float speed = 0;

                    if (previousLocation != null) {
                        float distance = previousLocation.distanceTo(location);
                        long timeElapsed = (currentTime - previousTime) / 1000;
                        speed = (timeElapsed > 0) ? (distance / timeElapsed) : 0;
                    }

                    previousLocation = location;
                    previousTime = currentTime;

                    if (locationUpdateCallback != null) {
                        locationUpdateCallback.onLocationUpdated(location, speed);
                    }
                }
            }
        };
    }

    public void setLocationUpdateCallback(LocationUpdateCallback callback) {
        this.locationUpdateCallback = callback;
    }

    public void startTracking() {
        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            return;
        }
        fusedLocationClient.requestLocationUpdates(LocationRequest.create(), locationCallback, null);
    }

    public void stopTracking() {
        fusedLocationClient.removeLocationUpdates(locationCallback);
    }
}
