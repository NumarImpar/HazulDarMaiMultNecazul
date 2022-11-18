package org.firstinsipres.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class JustPark extends LinearOpMode {
    public DcMotorEx LF, LB, RF, RB; 
    private OpenCvWebcam webcam;
    private AprilTagsPipeline pipeline = new AprilTagsPipeline(2, new int[] {0, 1, 2});
    private int actualTarget = -1;
    
    private void _init() {
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
	
	LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	
	_initDetection();
    }
    private void _initDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
	webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "eye"), cameraMonitorViewId);

	webcam.setMillisecondsPermissionTimeout(3000);
	webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
	    @Override
	    public void onOpened(){
	        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
		try{Thread.sleep(1000);}catch (InterruptedException e){;;}
	        webcam.setPipeline(pipeline);
	    }

	    @Override
	    public void onError(int errorCode){
	        telemetry.addLine(String.format("canera failed to open: errcode %d", errorCode));  
		telemetry.update();
	    }
	});
    }

    public void park(int target) {
	if (target < 0){
	    // detection failed so parks in terminal
	    LF.setPower(-0.3);
	    LB.setPower(0.3);
	    RF.setPower(0.3);
	    RB.setPower(-0.3);
	} else {
            LF.setPower(0.3);
            LB.setPower(0.3);
            RB.setPower(0.3);
            RF.setPower(0.3);
        }
	try {
	    Thread.sleep(300);
	} catch (InterruptedException e){;;}
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);

	if (target == 0){
            LF.setPower(-0.3);
            LB.setPower(0.3);
            RB.setPower(0.3);
            RF.setPower(-0.3);
	} else if (target == 1){
            ;; //pass	
	} else if (target == 2){
            LF.setPower(0.3);
            LB.setPower(-0.3);
            RB.setPower(-0.3);
            RF.setPower(0.3);
	}
	try {
	    Thread.sleep(300);
	} catch (InterruptedException e){;;}
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        RF.setPower(0);
    }

    @Override
    public void runOpMode(){
        _init();
	waitForStart();

	while(true){
	    actualTarget = pipeline.targetFound;
	   
            telemetry.addLine(String.format("target: %d", actualTarget));
	    if (pipeline._e != null) {
	        telemetry.addLine(String.format("unknown error: %s", pipeline._e.getMessage()));
	     }
	     telemetry.update();
	     if (pipeline.killThis){
	        webcam.stopStreaming();
	        webcam.closeCameraDevice();
	        try{pipeline.kill();} catch (Exception e){;;}
		break;
	     }
	}
	  park(actualTarget);

    }
}
