package com.example.movedistance;

import org.pytorch.IValue;
import org.pytorch.Module;
import org.pytorch.Tensor;
import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class PyTorchHelper {
    private static final String TAG = "PyTorchHelper";
    private static final String MODEL_FILENAME = "model.pt"; // 🔹 고정된 모델 파일명 상수화
    private Module model;

    public PyTorchHelper(Context context) {
        try {
            String modelPath = assetFilePath(context); // 🔹 model.pt을 직접 사용
            model = Module.load(modelPath);
        } catch (IOException e) {
            Log.e(TAG, "파일 복사 오류: " + e.getMessage(), e);
        } catch (Exception e) {
            Log.e(TAG, "모델 로드 실패: " + e.getMessage(), e);
        }
    }

    /**
     * Assets에서 내부 저장소로 모델 파일 복사 후 경로 반환
     */
    private String assetFilePath(Context context) throws IOException {
        File file = new File(context.getFilesDir(), MODEL_FILENAME);
        if (file.exists()) {
            return file.getAbsolutePath(); // 이미 존재하면 경로 반환
        }

        try (InputStream is = context.getAssets().open(MODEL_FILENAME);
             FileOutputStream fos = new FileOutputStream(file)) {

            byte[] buffer = new byte[4096]; // 버퍼 크기 최적화
            int read;
            while ((read = is.read(buffer)) != -1) {
                fos.write(buffer, 0, read);
            }
            fos.flush();
        }

        return file.getAbsolutePath();
    }

    /**
     * 모델 예측 실행
     */
    public float[] predict(float[] inputData) {
        if (model == null) {
            throw new IllegalStateException("PyTorch 모델이 로드되지 않았습니다.");
        }

        // 입력 데이터 크기 자동 설정
        long[] shape = new long[]{1, inputData.length};
        Tensor inputTensor = Tensor.fromBlob(inputData, shape);
        Tensor outputTensor = model.forward(IValue.from(inputTensor)).toTensor();
        return outputTensor.getDataAsFloatArray();
    }
}
