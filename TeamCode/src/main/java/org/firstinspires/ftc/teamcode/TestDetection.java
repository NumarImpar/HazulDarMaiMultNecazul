package org.firstinsipres.ftc.teamcode;

import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class TestDetection extends LinearOpMode {

    private OpenCvWebcam webcam;
    private AprilTagsPipeline pipeline = new AprilTagsPipeline(3, new int[] {0, 312, 2});
    private int actualTarget = -1;
    
    private void _init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
	webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "eye"), cameraMonitorViewId);

	webcam.setMillisecondsPermissionTimeout(3000);
	webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
	    @Override
	    public void onOpened(){
	        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
		try{Thread.sleep(3000);}catch (InterruptedException e){;}
	        webcam.setPipeline(pipeline);
	    }

	    @Override
	    public void onError(int errorCode){
	        telemetry.addLine(String.format("canera failed to open: errcode %d", errorCode));  
		telemetry.update();
	    }
	});
    }

    @Override
    public void runOpMode(){
        _init();
	ControllerInput controller1 = new ControllerInput(gamepad1);
	waitForStart();

	while(opModeIsActive()){
           if (gamepad1.square){
	       telemetry.clearAll();
	   }

	   if (controller1.AOnce()){
	       pipeline.setDecimation(pipeline.getDecimation() + 0.1f);
	   }

	   if (controller1.BOnce()){
	       pipeline.setDecimation(pipeline.getDecimation() - 0.1f);
	   }

	   telemetry.addLine(String.format("decimation: %f.2", pipeline.getDecimation()));
	   telemetry.addLine(String.format("ids: %d %d %d", pipeline.__ids[0], pipeline.__ids[1], pipeline.__ids[2]));
	   telemetry.addLine(String.format("targets: %d %d %d", pipeline._detectionIds[0], pipeline._detectionIds[1], pipeline._detectionIds[2]));
	   telemetry.addLine(String.format("target found: %d", pipeline.targetFound));

	   if (pipeline.targetFound != -1 && actualTarget == -1){
	       actualTarget = pipeline.targetFound;
	   }
	   telemetry.update();
	}

	webcam.stopStreaming();
	webcam.closeCameraDevice();
	pipeline.kill();
    }
}
