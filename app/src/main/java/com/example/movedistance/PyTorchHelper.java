package com.example.movedistance;

import android.content.Context;
import android.util.Log;
import android.widget.Toast;
import org.pytorch.IValue;
import org.pytorch.Module;
import org.pytorch.Tensor;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;

public class PyTorchHelper {
    private static final String TAG = "PyTorchHelper";
    private static final String MODEL_FILENAME = "model.pt"; // 🔹 모델 파일명
    private Module model;

    public PyTorchHelper(Context context) {
        try {
            String modelPath = assetFilePath(context);
            model = Module.load(modelPath);
            Log.d(TAG, "PyTorch 모델이 성공적으로 로드됨.");
        } catch (IOException e) {
            Log.e(TAG, "파일 복사 오류: " + e.getMessage(), e);
            Toast.makeText(context, "PyTorch 모델 로드 실패", Toast.LENGTH_LONG).show();
        } catch (Exception e) {
            Log.e(TAG, "모델 로드 실패: " + e.getMessage(), e);
            Toast.makeText(context, "PyTorch 모델 로드 중 오류 발생", Toast.LENGTH_LONG).show();
        }
    }

    /**
     * 📌 Assets에서 내부 저장소로 모델 파일 복사 후 경로 반환
     */
    private String assetFilePath(Context context) throws IOException {
        File file = new File(context.getFilesDir(), MODEL_FILENAME);
        if (file.exists()) {
            return file.getAbsolutePath(); // 이미 존재하면 경로 반환
        }

        try (InputStream is = context.getAssets().open(MODEL_FILENAME);
             FileOutputStream fos = new FileOutputStream(file)) {

            byte[] buffer = new byte[4096];
            int read;
            while ((read = is.read(buffer)) != -1) {
                fos.write(buffer, 0, read);
            }
            fos.flush();
        }
        return file.getAbsolutePath();
    }

    /**
     * 📌 모델 예측 실행
     */
    public float[] predict(Tensor inputTensor) {
        try {
            // ✅ 입력 크기 로깅 (디버깅 용도)
            Log.d("PyTorch", "✅ 입력 텐서 크기: " + Arrays.toString(inputTensor.shape()));

            // ✅ 모델 실행
            Tensor outputTensor = model.forward(IValue.from(inputTensor)).toTensor();

            // ✅ 출력 크기 로깅
            Log.d("PyTorch", "✅ 출력 텐서 크기: " + Arrays.toString(outputTensor.shape()));

            return outputTensor.getDataAsFloatArray();
        } catch (Exception e) {
            Log.e("PyTorch", "❌ 모델 예측 중 오류 발생: " + e.getMessage());
            return new float[0];  // 예측 실패 시 빈 배열 반환하여 앱 크래시 방지
        }
    }

}
