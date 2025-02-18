package com.example.movedistance;

import android.content.Context;
import android.graphics.Color;
import org.osmdroid.config.Configuration;
import org.osmdroid.tileprovider.tilesource.TileSourceFactory;
import org.osmdroid.util.GeoPoint;
import org.osmdroid.views.MapView;
import org.osmdroid.views.overlay.Polyline;
import org.osmdroid.views.overlay.mylocation.GpsMyLocationProvider;
import org.osmdroid.views.overlay.mylocation.MyLocationNewOverlay;
import java.util.ArrayList;
import java.util.List;

import android.os.Environment;
import org.osmdroid.tileprovider.tilesource.XYTileSource;

import java.io.File;

public class OfflineMapManager {
    private final MapView mapView;
    private final MyLocationNewOverlay myLocationOverlay;
    private final List<GeoPoint> routePoints = new ArrayList<>();

    public OfflineMapManager(Context context, MapView mapView) {
        this.mapView = mapView;
        Configuration.getInstance().load(context, context.getSharedPreferences("osmdroid", Context.MODE_PRIVATE));

        File mapFile = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/osmdroid/south-korea.map");
        if (mapFile.exists()) {
            mapView.setTileSource(new XYTileSource(
                    "OfflineMap", 0, 18, 256, ".map", new String[]{}
            ));
        } else {
            mapView.setTileSource(TileSourceFactory.MAPNIK); // 온라인 지도 사용
        }

        mapView.setMultiTouchControls(true);

        mapView.setUseDataConnection(true);
        mapView.getController().setZoom(15.0);

        myLocationOverlay = new MyLocationNewOverlay(new GpsMyLocationProvider(context), mapView);
        myLocationOverlay.enableMyLocation();
        mapView.getOverlays().add(myLocationOverlay);
    }

    public void updateLocation(GeoPoint newPoint, float speed) {
        if (!routePoints.isEmpty()) {
            GeoPoint lastPoint = routePoints.get(routePoints.size() - 1);
            Polyline segment = new Polyline();
            segment.setColor(getSpeedColor(speed));
            segment.setWidth(5.0f);
            segment.addPoint(lastPoint);
            segment.addPoint(newPoint);
            mapView.getOverlays().add(segment);

        }

        routePoints.add(newPoint);
        mapView.getController().setCenter(newPoint);
        mapView.invalidate();
    }

    private int getSpeedColor(float speed) {
        if (speed < 1.5) return Color.BLUE;
        if (speed < 3.0) return Color.GREEN;
        if (speed < 5.0) return Color.YELLOW;
        return Color.RED;
    }
}
