package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.SignalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Detection test - SignalPipeline", group = "Testing")
public class DetectionTest extends LinearOpMode {

    public final SignalPipeline pipeline = new SignalPipeline(1.5F, new int[] {0, 1, 2});
    protected OpenCvWebcam webcam;

    public int target = -1;
    protected boolean cameraOK = true;

    @Override
    public void runOpMode() throws InterruptedException {
        initDetection();

        if (cameraOK) {
            telemetry.addLine( "ok");
        } else {
            telemetry.addLine("perform reinitialization!");
        }
        telemetry.update();

        pipeline.startAprilTagDetection();

        telemetry.addLine("press start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            target = pipeline.targetFound;
            telemetry.addData("target", target);
            telemetry.update();
        }

        pipeline.stopAprilTagDetection();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }



    private void initDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "eye"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(600); //3000
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){

            @Override
            public void onOpened(){
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                try{
                    Thread.sleep(1000); // always wait before pressing start
                } catch (InterruptedException e){
                    e.printStackTrace();
                }

                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode){
                telemetry.addData("camera failed to open:", errorCode);
                telemetry.update();
                cameraOK = false;
            }
        });
    }
}
