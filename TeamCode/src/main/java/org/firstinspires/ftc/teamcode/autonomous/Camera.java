package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

public class Camera {
    public OpenCvWebcam webcam;
    public boolean ok = true;

    public Camera(@NonNull HardwareMap _hwMap){
        int cameraMonitorViewId = _hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", _hwMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(_hwMap.get(WebcamName.class, "eye"), cameraMonitorViewId);
        webcam.setMillisecondsPermissionTimeout(600); //3000
    }

    public void initDetection(OpenCvPipeline _pipeline, Telemetry _telemetry){
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened(){
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                try{
                    // wait before start in ALL OPMODES THAT MAKE USE OF THIS
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                webcam.setPipeline(_pipeline);
            }

            @Override
            public void onError(int errorCode){
                _telemetry.addData("camera failed to open", errorCode);
                _telemetry.update();
                ok = false;
            }
        });
    }

    public void close() {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
