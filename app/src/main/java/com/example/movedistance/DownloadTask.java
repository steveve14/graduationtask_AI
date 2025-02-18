package com.example.movedistance;

import android.app.ProgressDialog;
import android.content.Context;
import android.os.AsyncTask;
import android.os.Environment;
import android.util.Log;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;

public class DownloadTask extends AsyncTask<String, Integer, String> {
    private static final String TAG = "DownloadTask";
    private final Context context;
    private ProgressDialog progressDialog;
    private final String savePath;

    public DownloadTask(Context context) {
        this.context = context;
        this.savePath = Environment.getExternalStorageDirectory().getAbsolutePath() + "/osmdroid/";
    }

    @Override
    protected void onPreExecute() {
        super.onPreExecute();
        progressDialog = new ProgressDialog(context);
        progressDialog.setTitle("지도 다운로드 중...");
        progressDialog.setMessage("잠시만 기다려 주세요.");
        progressDialog.setIndeterminate(true);
        progressDialog.setCancelable(false);
        progressDialog.show();
    }

    @Override
    protected String doInBackground(String... urls) {
        String fileUrl = urls[0]; // 다운로드할 파일 URL
        String fileName = "south-korea.map"; // 저장할 파일 이름
        File directory = new File(savePath);
        if (!directory.exists()) {
            directory.mkdirs();
        }

        File outputFile = new File(directory, fileName);
        if (outputFile.exists()) {
            return "이미 다운로드됨";
        }

        try {
            URL url = new URL(fileUrl);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.connect();

            if (connection.getResponseCode() != HttpURLConnection.HTTP_OK) {
                return "서버 응답 코드: " + connection.getResponseCode();
            }

            InputStream inputStream = new BufferedInputStream(connection.getInputStream());
            FileOutputStream fileOutputStream = new FileOutputStream(outputFile);
            byte[] data = new byte[1024];
            int count;
            while ((count = inputStream.read(data)) != -1) {
                fileOutputStream.write(data, 0, count);
            }

            fileOutputStream.flush();
            fileOutputStream.close();
            inputStream.close();
            return "다운로드 완료!";
        } catch (Exception e) {
            Log.e(TAG, "다운로드 오류", e);
            return "다운로드 실패";
        }
    }

    @Override
    protected void onPostExecute(String result) {
        super.onPostExecute(result);
        if (progressDialog.isShowing()) {
            progressDialog.dismiss();
        }
        Log.d(TAG, result);
    }
}
