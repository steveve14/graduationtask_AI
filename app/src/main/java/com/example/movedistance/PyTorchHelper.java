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
    public float[] predict(float[] inputData) {
        if (model == null) {
            throw new IllegalStateException("PyTorch 모델이 로드되지 않았습니다.");
        }

        // ✅ 모델이 요구하는 입력 크기로 설정 (예: 340개 값)
        int expectedInputSize = 340;
        float[] resizedInput = new float[expectedInputSize];

        // 입력 데이터 크기 확인 후 조정
        if (inputData.length < expectedInputSize) {
            System.arraycopy(inputData, 0, resizedInput, 0, inputData.length);
        } else {
            System.arraycopy(inputData, 0, resizedInput, 0, expectedInputSize);
        }

        // 🔹 입력 데이터 PyTorch Tensor 변환
        long[] shape = new long[]{1, expectedInputSize};
        Tensor inputTensor = Tensor.fromBlob(resizedInput, shape);

        // 🔹 PyTorch 모델 실행
        Tensor outputTensor = model.forward(IValue.from(inputTensor)).toTensor();
        return outputTensor.getDataAsFloatArray();
    }
}
