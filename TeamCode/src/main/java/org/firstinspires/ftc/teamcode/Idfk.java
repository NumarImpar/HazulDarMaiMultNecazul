package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class Idfk extends LinearOpMode {

    public SampleMecanumDrive drive;

    public ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        timer = new ElapsedTime();

        waitForStart();

        while(opModeIsActive()){
	    drive.setMotorPowers(-0.4, 0.4, 0.4, -0.4);
	    if (timer.seconds() < 1){
	        continue;
	    }
            drive.setMotorPowers(0,0,0,0);
            break;
        }
    }
}
