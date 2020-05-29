package jp.ac.titech.itpro.sdl.resist;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import org.apache.commons.math3.complex.Quaternion;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import androidx.appcompat.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity implements SensorEventListener {
    private final static String TAG = MainActivity.class.getSimpleName();

    private RotationView rotationView;
    private SensorManager manager;
    private Sensor gyroscope;

    private long previousTime = -1;

    /// アプリ起動時のローカル座標系を基準としたグローバル座標系(端末の姿勢)
    //
    // 端末本体を少しでも3次元的に回転させると計測される角速度の座標系がずれて
    // 針が中央に戻らなくなるので、角速度を取得するたびに Quaternion を用いて
    // ローカル座標軸を回転させる
    private Quaternion axisX = Quaternion.I;
    private Quaternion axisY = Quaternion.J;
    private Quaternion axisZ = Quaternion.K;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Log.d(TAG, "onCreate");

        rotationView = findViewById(R.id.rotation_view);

        manager = (SensorManager) getSystemService(SENSOR_SERVICE);
        if (manager == null) {
            Toast.makeText(this, R.string.toast_no_sensor_manager, Toast.LENGTH_LONG).show();
            finish();
            return;
        }
        gyroscope = manager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        if (gyroscope == null) {
            Toast.makeText(this, R.string.toast_no_gyroscope, Toast.LENGTH_LONG).show();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.d(TAG, "onResume");
        manager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_FASTEST);
    }

    @Override
    protected void onPause() {
        super.onPause();
        Log.d(TAG, "onPause");
        manager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        long currentTime = event.timestamp;

        if (previousTime > 0) {
            float dt = (float) (currentTime - previousTime) / 1_000_000_000.0f;

            float omegaXHalf = event.values[0] / 2.0f * dt;  // x-axis angular velocity (rad/sec)
            float omegaYHalf = event.values[1] / 2.0f * dt;  // y-axis angular velocity (rad/sec)
            float omegaZHalf = event.values[2] / 2.0f * dt;  // z-axis angular velocity (rad/sec)

            // ローカル座標系における各軸上の回転を表すクォータニオン
            Quaternion qx = new Quaternion(
                    Math.cos(omegaXHalf),
                    axisX.getQ1() * Math.sin(omegaXHalf),
                    axisX.getQ2() * Math.sin(omegaXHalf),
                    axisX.getQ3() * Math.sin(omegaXHalf));
            Quaternion qy = new Quaternion(
                    Math.cos(omegaYHalf),
                    axisY.getQ1() * Math.sin(omegaYHalf),
                    axisY.getQ2() * Math.sin(omegaYHalf),
                    axisY.getQ3() * Math.sin(omegaYHalf));
            Quaternion qz = new Quaternion(
                    Math.cos(omegaZHalf),
                    axisZ.getQ1() * Math.sin(omegaZHalf),
                    axisZ.getQ2() * Math.sin(omegaZHalf),
                    axisZ.getQ3() * Math.sin(omegaZHalf));

            Quaternion q = Quaternion.multiply(Quaternion.multiply(qx, qy), qz);
            Quaternion qInv = q.getInverse();

            // ローカル座標系の更新
            axisX = Quaternion.multiply(Quaternion.multiply(q, axisX), qInv);
            axisY = Quaternion.multiply(Quaternion.multiply(q, axisY), qInv);
            axisZ = Quaternion.multiply(Quaternion.multiply(q, axisZ), qInv);

            // ローカル座標系の xy 平面上の単位円の円周上の点のうちグローバル座標系における
            // y 座標がもっとも大きいものを点 P とする時の、ローカル座標系の y 軸とベクトル
            // OP がなす角度を求める(端末の上方向と針のなす角度)
            Vector3D yVec = new Vector3D(axisY.getVectorPart());
            Vector3D zVec = new Vector3D(axisZ.getVectorPart());
            Vector3D yMax = zVec.crossProduct(Vector3D.PLUS_J).crossProduct(zVec).normalize();

            double prod = yVec.dotProduct(yMax);

            // 針が y 軸の左にあるか右にあるかによって符号を反転
            double sign = yVec.crossProduct(yMax).dotProduct(zVec) > 0 ? 1 : -1;

            // 針と端末上方向がなす角度を [-PI, PI] で計算
            double direction = sign * Math.acos(prod);

            // 打ち消す角度を求めるので、符号を反転
            rotationView.setDirection(-direction);
        }
        previousTime = currentTime;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        Log.d(TAG, "onAccuracyChanged: accuracy=" + accuracy);
    }
}
