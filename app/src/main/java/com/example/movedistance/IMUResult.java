package com.example.movedistance;

public class IMUResult {
    private double[][] x, y, z;
    private double[][] features;
    private double[][][] rawData;

    public IMUResult(double[][] features) {
        this.features = features;
    }

    public IMUResult(double[][] x, double[][] y, double[][] z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public IMUResult(double[][][] rawData) {
        this.rawData = rawData;
    }

    public double[][] getFeatures() {
        if (features == null) {
            System.err.println("⚠ Warning: getFeatures() called but features is null.");
        }
        return features;
    }

    public double[][] getX() {
        return x;
    }

    public double[][] getY() {
        return y;
    }

    public double[][] getZ() {
        return z;
    }

    public double[][][] getRawData() {
        return rawData;
    }
}
