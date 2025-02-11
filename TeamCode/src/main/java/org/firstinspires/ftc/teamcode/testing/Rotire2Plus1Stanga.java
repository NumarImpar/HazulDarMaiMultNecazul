package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.Drive;
import org.firstinspires.ftc.teamcode.util.Automatisms;
import org.firstinspires.ftc.teamcode.util.SignalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous
public class Rotire2Plus1Stanga extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();
    public final SignalPipeline pipeline = new SignalPipeline(1.75F, new int[] {0, 1, 2});
    protected OpenCvWebcam webcam;

    public int target = -1;
    protected boolean cameraOK = true;

    public SampleMecanumDrive drive;
    private Automatisms automatism;

    public Lifter lifter;
    public Intake intake;

    public Lifter.LIFTER_LEVEL currentLifterState;
    public Intake.INTAKE_STATE currentIntakeState;

    protected Thread intakeThread, lifterThread;

    static Pose2d initialPoseLeft = new Pose2d(-36, -63, Math.toRadians(90));

    private void initDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "eye"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(3000);
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

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        initDetection();
        timer.reset();

        automatism = new Automatisms(lifter, intake);

        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        currentIntakeState = Intake.INTAKE_STATE.INIT;

        intake.spinFor(0, 50, 0.1); //bug

        TrajectorySequence toHighPreload = drive.trajectorySequenceBuilder(initialPoseLeft)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.swingSetTargetTicks(200, -100);
                })
                //to high preload
//                .forward(30)
//                .splineToSplineHeading(new Pose2d(-48, -11, Math.toRadians(180)), Math.toRadians(170))
//                .setReversed(true)


                .splineToSplineHeading(new Pose2d(-48, -35, Math.toRadians(180)), Math.toRadians(170))
                .setReversed(true)

                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, Intake.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                    intake.spinFor(2000, 800, 0.6);
                })
//                .splineToLinearHeading(new Pose2d(-28, -1, Math.toRadians(235)), Math.toRadians(45))
                // --

                .splineToLinearHeading(new Pose2d(-29, -1, Math.toRadians(213)), Math.toRadians(50))

                .build();

        TrajectorySequence toStack1 = drive.trajectorySequenceBuilder(toHighPreload.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.LOW, Intake.INTAKE_STATE.OUTSIDE);
                    lifter.setTargetTicks(700, 550);
                    intake.swingSetTargetTicks(0, 1000);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;

                    intake.spinFor(1600, 1800, -0.6);
                    intake.swingSetTargetTicks(0, 2000);

                    lifter.setTargetTicks(2500, 1900);
                    currentLifterState = Lifter.LIFTER_LEVEL.MID;
                })

                .setReversed(false)

                .splineToSplineHeading(new Pose2d(-58.1, -12, Math.toRadians(180)), Math.toRadians(177)) //195
               // .lineTo(new Vector2d(-57, -12))
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(toHighPreload.end())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
//                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.LOW, Intake.INTAKE_STATE.OUTSIDE);
                    lifter.setTargetTicks(700, 550);
                    intake.swingSetTargetTicks(0, 1000);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;

                    intake.spinFor(1600, 1800, -0.6);
                    intake.swingSetTargetTicks(0, 2000);

                    lifter.setTargetTicks(2500, 1900);
                    currentLifterState = Lifter.LIFTER_LEVEL.MID;
                })

                .setReversed(false)

                .splineToSplineHeading(new Pose2d(-57.7, -12, Math.toRadians(180)), Math.toRadians(177)) //195
                // .lineTo(new Vector2d(-57, -12))
                .build();

        TrajectorySequence toHighFromStack1 = drive.trajectorySequenceBuilder(toStack1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, Intake.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;
                    intake.spinFor(2000, 600, 0.6);
                })

//                .setReversed(true)
//                .back(20)
//                .splineToSplineHeading(new Pose2d(-28, -1, Math.toRadians(270 - 47)), Math.toRadians(65))

                .setReversed(true)
                .back(10)
                .splineToSplineHeading(new Pose2d(-27.1, -1, Math.toRadians(217)), Math.toRadians(65))

                .build();

        TrajectorySequence toHighFromStack2 = drive.trajectorySequenceBuilder(toStack2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.HIGH, Intake.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;
                    intake.spinFor(2000, 600, 0.6);
                })

//                .setReversed(true)
//                .back(20)
//                .splineToSplineHeading(new Pose2d(-28, -1, Math.toRadians(270 - 47)), Math.toRadians(65))

                .setReversed(true)
                .back(10)
                .splineToSplineHeading(new Pose2d(-27.1, -1, Math.toRadians(217)), Math.toRadians(65))

                .build();

        if (cameraOK) {
            telemetry.addLine( "ok");
        } else {
            telemetry.addLine("perform reinitialization!");
        }

        telemetry.update();

        waitForStart();

        if (cameraOK) {
            pipeline.startAprilTagDetection();
            timer.reset();

            while (target < 0 && timer.milliseconds() < 1500){
                target = pipeline.targetFound;
            }
        }

        pipeline.stopAprilTagDetection();
        webcam.stopStreaming();
        webcam.closeCameraDevice();


        telemetry.addData("target:", target);
        telemetry.update();

        intakeThread = new Thread(intake);
        lifterThread = new Thread(lifter);

        lifterThread.start();
        intakeThread.start();

        drive.setPoseEstimate(initialPoseLeft);
        drive.followTrajectorySequence(toHighPreload);
        sleep(1000);
        drive.followTrajectorySequence(toStack1);
        sleep(1600);
        drive.followTrajectorySequence(toHighFromStack1);
        sleep(1000);
        drive.followTrajectorySequence(toStack2);
        sleep(1600);
        drive.followTrajectorySequence(toHighFromStack2);
        sleep(1000);

        TrajectorySequence toPark1 = drive.trajectorySequenceBuilder(toHighFromStack2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, Intake.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                })

                .setReversed(false)

                .splineToSplineHeading(new Pose2d(-57.7, -12, Math.toRadians(180)), Math.toRadians(177))
//                .setReversed(false)
//                .splineToSplineHeading(new Pose2d(-57, -12, Math.toRadians(180)), Math.toRadians(195))
                .build();

        TrajectorySequence toPark2 = drive.trajectorySequenceBuilder(toHighFromStack2.end())
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, Intake.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                })
                .splineToSplineHeading(new Pose2d(-37, -25, Math.toRadians(-90)), Math.toRadians(270))

                .build();

        TrajectorySequence toPark3 = drive.trajectorySequenceBuilder(toHighFromStack2.end())
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, Intake.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                })
//                .splineToLinearHeading(new Pose2d(-15, -13, Math.toRadians(180)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(-13, -15, Math.toRadians(180)), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-13, -13.5, Math.toRadians(0)), Math.toRadians(-180))

                .build();
        if (target == 0){
            drive.followTrajectorySequence(toPark1);
        }
        else if(target == 2){
            drive.followTrajectorySequence(toPark3);
        }
        else {
            drive.followTrajectorySequence(toPark2);
        }

        lifter.kill = true;
        intake.kill = true;

    }
}
