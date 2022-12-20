package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.OpenCV;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

import static org.openftc.easyopencv.OpenCvCameraFactory.*;

@Autonomous
public class AutoBlue133_138 extends LinearOpMode {

    //drivetrain
    SampleMecanumDrive drive;

    //servo
    //Servo

    //motoare
    //DcMotor

    private DcMotor lift_right, lift_left;

    private Servo link1, link2;
    private CRServo intake1, intake2;


    //opencv
    private OpenCvCamera webcam;
    private UltimateGoalPipeline pipeline;

    //opmode
    @Override
    public void runOpMode() throws InterruptedException {

        //init
        initHardware();
        initOpenCv();
        drive = new SampleMecanumDrive(hardwareMap);
        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("case: " , pipeline.getAnalysis());
            telemetry.update();
        }
        waitForStart();

        if(isStopRequested()) return;
        cazulFOUR();
//        makeCase();
    }

    //alege cazul
    private void makeCase() {

        if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.FOUR)
            cazulFOUR();

        else if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.ONE)
            cazulONE();

        else if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.ZERO)
            cazulZERO();
    }



    //cazuri
    private void cazulZERO(){}
    private void cazulONE(){}
    private void cazulFOUR(){

        Trajectory shootRing  = drive.trajectoryBuilder(new Pose2d(0,0))
                .lineToLinearHeading(new Pose2d(50,0,Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(65 ,DriveConstants.TRACK_WIDTH)
                                )
                        ),new ProfileAccelerationConstraint(40)
                )
                .addTemporalMarker(0.01,() ->{
                })
                .build();

        Trajectory t2  = drive.trajectoryBuilder(new Pose2d(50,0,Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(50,20,Math.toRadians(-90)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(65 ,DriveConstants.TRACK_WIDTH)
                                )
                        ),new ProfileAccelerationConstraint(40)
                )
                .addTemporalMarker(0.01,() ->{
                })
                .build();

        drive.followTrajectory(shootRing);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(t2);

    }


    //inituri
    private void initOpenCv(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam =  getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"),cameraMonitorViewId);
        pipeline = new UltimateGoalPipeline(telemetry);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                ;;
            }
        });
    }

    private void initHardware(){
        lift_left = hardwareMap.get(DcMotorEx.class, "lifterLeft");
        lift_right = hardwareMap.get(DcMotorEx.class, "lifterRight");

        lift_right.setDirection(DcMotorSimple.Direction.REVERSE);


        lift_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO: link1- left
        //TODO: link2-right

        link1 = hardwareMap.get(Servo.class, "leftServoSwing");
        link2 = hardwareMap.get(Servo.class, "rightServoSwing");


        intake1 = hardwareMap.get(CRServo.class, "leftCRServoIntake");
        intake2 = hardwareMap.get(CRServo.class, "rightCRServoIntake");

        link2.setDirection(Servo.Direction.REVERSE);
        intake2.setDirection(CRServo.Direction.REVERSE);

    }


    //clasa openCv
    public static class UltimateGoalPipeline extends OpenCvPipeline
    {
        Telemetry telemetry;

        public UltimateGoalPipeline(Telemetry telemetry){
            this.telemetry = telemetry;

        }

        public enum UltimateGoalRings
        {
            FOUR,
            ONE,
            ZERO
        }


        static final Scalar RED = new Scalar(255, 0, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(461,531);

        static final int REGION_WIDTH = 120;
        static final int REGION_HEIGHT = 80;
        static final int FOUR_RING_THRESHOLD = 105; // 93    124 for ZERO
        static final int ONE_RING_THRESHOLD = 115; //104


        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg;

        private volatile UltimateGoalRings position = UltimateGoalRings.ZERO;

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb );
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg = (int) Core.mean(region1_Cb).val[0];
            telemetry.addData("avg: ",avg);
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            if(avg <= FOUR_RING_THRESHOLD)
            {
                position = UltimateGoalRings.FOUR;

            }
            else if(avg <= ONE_RING_THRESHOLD)
            {
                position = UltimateGoalRings.ONE;

            }
            else
            {
                position = UltimateGoalRings.ZERO;
            }

            return input;
        }

        public UltimateGoalRings getAnalysis()
        {
            return position;
        }
    }}
