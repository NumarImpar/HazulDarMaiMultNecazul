package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Automatisms;
import org.firstinspires.ftc.teamcode.util.SignalPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.util.Arrays;

@Disabled
@Autonomous (name = "testStanga")
public class StangaHighMid extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();
    public final SignalPipeline pipeline = new SignalPipeline(1.5F, new int[]{0, 1, 2});
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

    static Pose2d initialPoseLeft = new Pose2d(-36, -63, Math.toRadians(270));

    private void initDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "eye"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(3000); //3000
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                try {
                    Thread.sleep(1000); // always wait before pressing start
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initDetection();
        timer.reset();

        automatism = new Automatisms(lifter, intake);

        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        currentIntakeState = Intake.INTAKE_STATE.INIT;

        intake.spinFor(0, 50, 0.1); //bug

        // Creating trajectories
        TrajectorySequence toHighPreload = toHighPreload();

        TrajectorySequence toCone1 = toConeOne(toHighPreload);
        TrajectorySequence toMid1 = toMidOne(toCone1);

        TrajectorySequence toCone2 = toConeTwo(toMid1);
        TrajectorySequence toMid2 = toMidTwo(toCone2);

        TrajectorySequence toCone3 = toConeThree(toMid2);
        TrajectorySequence toMid3 = toMidThree(toCone3);

        TrajectorySequence toCone4 = toConeFour(toMid3);
        TrajectorySequence toMid4 = toMidFour(toCone4);

        TrajectorySequence toPark1 = toParkOne(toMid4);
        TrajectorySequence toPark2 = toParkTwo(toMid4);
        TrajectorySequence toPark3 = toParkThree(toMid4);

        pipeline.startAprilTagDetection();

        if (cameraOK) {
            telemetry.addLine("Created trajectories");
            telemetry.addLine("Webcam Ok");
            telemetry.addLine("Ready! Press Play");
        } else {
            telemetry.addLine("Webcam failed, please RESTART!");
            telemetry.update();
            sleep(1000);
        }

        telemetry.update();

        waitForStart();

        target = pipeline.targetFound;

        pipeline.stopAprilTagDetection();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        telemetry.addData("Sleeve", target + 1);
        telemetry.update();

        intakeThread = new Thread(intake);
        lifterThread = new Thread(lifter);

        lifterThread.start();
        intakeThread.start();

        drive.setPoseEstimate(initialPoseLeft);

//        intake.setCRServosPow(-0.1);


//        lifter.setTargetTicks(200, Lifter.LIFTER_LEVEL.LOW.ticks);
//        intake.swingSetTargetTicks(200, Intake.INTAKE_STATE.INSIDE.ticks);
//        currentLifterState = Lifter.LIFTER_LEVEL.LOW;
//        currentIntakeState = Intake.INTAKE_STATE.INSIDE;

//        intake.paConGoToPositionThread(0.8, 200);  // IF IT AIN'T BROKE DON'T FIX IT

        drive.followTrajectorySequence(toHighPreload);
        sleep(50);
//        intake.paConInside(800);

        drive.followTrajectorySequence(toCone1);
        sleep(700);
//        intake.setCRServosPow(-0.1);
//
//        drive.followTrajectorySequence(toMid1);
//        sleep(50);
//
//        drive.followTrajectorySequence(toCone2);
//        sleep(700);
//
//        drive.followTrajectorySequence(toMid2);
//        sleep(50);
//
//        drive.followTrajectorySequence(toCone3);
//        sleep(700);
//
//        drive.followTrajectorySequence(toMid3);
//        sleep(50);
//
//        drive.followTrajectorySequence(toCone4);
//        sleep(700);
//
//        drive.followTrajectorySequence(toMid4);
//        sleep(50);
//
//        if(target == 0){
//            drive.followTrajectorySequence(toPark1);
//        } else if( target == 2){
//            drive.followTrajectorySequence(toPark3);
//        } else {
//            drive.followTrajectorySequence(toPark2);
//        }
//
//        sleep(200);

        while(opModeIsActive()){
            drive.update();
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("odo", Math.toDegrees(drive.getPoseEstimate().getHeading()));
            telemetry.addData("heading", Math.toDegrees(Angle.norm(drive.getRawExternalHeading())));
            telemetry.addData("target heading", Math.toDegrees(toHighPreload.end().getHeading()));

            telemetry.update();
        }

        lifter.finalLifterTicks = lifter.getCurrentPosition();
        intake.finalIntakeTicks = intake.getCurrentPosition();

        lifter.kill = true;
        intake.kill = true;
    }

    TrajectorySequence toHighPreload() {
        return drive.trajectorySequenceBuilder(initialPoseLeft)
//                .UNSTABLE_addTemporalMarkerOffset(0.44, () -> {
//                    lifter.setTargetTicks(200, Lifter.LIFTER_LEVEL.HIGH.ticks);
//                    intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.INSIDE.ticks);
//
//                    intake.paConInterpolateThread(600, 0.7, 0.45, 1000, 30); // IF IT AIN'T BROKE DON'T FIX IT
//
//                    currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
//                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
//
//                    intake.spinFor(2200, 800, 0.6);
//                })
//                .setReversed(false)
//
//                .lineToLinearHeading(new Pose2d(-34, -30, Math.toRadians(270)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(75, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(42))
//
//                .splineToSplineHeading(new Pose2d(-27.7, 0.6, Math.toRadians(201)), Math.toRadians(63.5),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(55, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(25)
//                )

                //vlad:

                .setReversed(true)

                .setAccelConstraint(new ProfileAccelerationConstraint(27))
                .setVelConstraint(new TranslationalVelocityConstraint(45))

                .lineToLinearHeading(new Pose2d(-36.5, -22, Math.toRadians(270)))
                .resetConstraints()

                .setAccelConstraint(new ProfileAccelerationConstraint(30))
                .setVelConstraint(new TranslationalVelocityConstraint(35))

                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(231)), Math.toRadians(53))
//                .lineToLinearHeading(new Pose2d(-25.2, -1.3, Math.toRadians(231)))

                .resetConstraints()

                .build();
    }

    TrajectorySequence toConeOne(@NonNull TrajectorySequence toHighPreload) {
        return drive.trajectorySequenceBuilder(toHighPreload.end())
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                    intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.OUTSIDE.ticks - 50);
//                    lifter.setTargetTicks(300, Lifter.LIFTER_LEVEL.LOW.ticks);
//
//                    lifter.setTargetTicks(1350, Lifter.LIFTER_LEVEL.LOW.ticks - 600);
//
//                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
//                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;
//                    intake.setCRServosPow(-0.5);
//                    lifter.setTargetTicks(2600, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
//                })
//                .setReversed(false)
//                .setVelConstraint(new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH))
//                .setAccelConstraint(new ProfileAccelerationConstraint(25))
//
//                .splineToSplineHeading(new Pose2d(-45, -12, Math.toRadians(180)), Math.toRadians(180))
//
//                .resetVelConstraint()
//                .resetAccelConstraint()
//
//                .splineToConstantHeading(new Vector2d(-57.5, -12), Math.toRadians(180))

                //vlad:

                .setReversed(false)

                .setAccelConstraint(new ProfileAccelerationConstraint(20))
                .setVelConstraint(new TranslationalVelocityConstraint(25))

                .splineToSplineHeading(new Pose2d(-45, -11.811, Math.toRadians(180)), Math.toRadians(180))

                .setAccelConstraint(new ProfileAccelerationConstraint(25))
                .setVelConstraint(new TranslationalVelocityConstraint(35))

                .splineToConstantHeading(new Vector2d(-57.5, -12), Math.toRadians(180))

                .resetConstraints()

                .waitSeconds(0.5)

                .setReversed(true)

                .splineToSplineHeading(new Pose2d(-25.2, -1.6, Math.toRadians(233)), Math.toRadians(53))

                .waitSeconds(0.5)
                .setReversed(false)

                .build();
    }

    TrajectorySequence toMidOne(@NonNull TrajectorySequence toCone1) {
        return drive.trajectorySequenceBuilder(toCone1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {

                    intake.swingSetTargetTicks(200, Intake.INTAKE_STATE.INSIDE.ticks);
                    lifter.setTargetTicks(700, Lifter.LIFTER_LEVEL.MID.ticks);

                    currentLifterState = Lifter.LIFTER_LEVEL.MID;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                    intake.spinFor(1900, 800, 0.8);
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-42, -13, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(30))

                .splineToSplineHeading(new Pose2d(-24.8, -21.3, Math.toRadians(116)), Math.toRadians(322),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(32)
                )

                .build();
    }

    TrajectorySequence toConeTwo(@NonNull TrajectorySequence toHigh1) {
        return drive.trajectorySequenceBuilder(toHigh1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    lifter.setTargetTicks(100, Lifter.LIFTER_LEVEL.MID.ticks);
                    intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.OUTSIDE.ticks);

                    lifter.setTargetTicks(850, Lifter.LIFTER_LEVEL.LOW.ticks - 600);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;

                    intake.setCRServosPow(-0.8);

                    lifter.setTargetTicks(2200, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-45, -13, Math.toRadians(180)), Math.toRadians(180),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(25)
                )
                .splineToConstantHeading(new Vector2d(-57.5, -14  ), Math.toRadians(180))
                .build();
    }

    TrajectorySequence toMidTwo(@NonNull TrajectorySequence toCone2) {
        return drive.trajectorySequenceBuilder(toCone2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.swingSetTargetTicks(100, Intake.INTAKE_STATE.INSIDE.ticks);
                    lifter.setTargetTicks(700, Lifter.LIFTER_LEVEL.MID.ticks);

                    currentLifterState = Lifter.LIFTER_LEVEL.MID;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                    intake.spinFor(2000, 800, 0.8);
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-42, -14, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(30))

                .splineToSplineHeading(new Pose2d(-24.5, -21.3, Math.toRadians(115)), Math.toRadians(322),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(30)
                )

                .build();
    }

    TrajectorySequence toConeThree(@NonNull TrajectorySequence toHigh2) {
        return drive.trajectorySequenceBuilder(toHigh2.end())
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    lifter.setTargetTicks(100, Lifter.LIFTER_LEVEL.MID.ticks);
                    intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.OUTSIDE.ticks);

                    lifter.setTargetTicks(800, Lifter.LIFTER_LEVEL.LOW.ticks - 700);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;

                    intake.setCRServosPow(-0.8);

                    lifter.setTargetTicks(2400, Lifter.LIFTER_LEVEL.LOW.ticks + 400);
                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-45, -14, Math.toRadians(180)), Math.toRadians(180),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(25)
                )
                .splineToConstantHeading(new Vector2d(-58.1, -13), Math.toRadians(180))
                .build();
    }

    TrajectorySequence toMidThree(@NonNull TrajectorySequence toCone3) {
        return drive.trajectorySequenceBuilder(toCone3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.swingSetTargetTicks(250, Intake.INTAKE_STATE.INSIDE.ticks);
                    lifter.setTargetTicks(700, Lifter.LIFTER_LEVEL.MID.ticks);

                    currentLifterState = Lifter.LIFTER_LEVEL.MID;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                    intake.spinFor(2000, 800, 0.8);
                })
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-42, -14, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(30))

                .splineToSplineHeading(new Pose2d(-23.7, -21.4, Math.toRadians(114)), Math.toRadians(319),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(30)
                )

                .build();
    }

    TrajectorySequence toConeFour(@NonNull TrajectorySequence toHigh3) {
        return drive.trajectorySequenceBuilder(toHigh3.end())
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    lifter.setTargetTicks(100, Lifter.LIFTER_LEVEL.MID.ticks);
                    intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.OUTSIDE.ticks);

                    lifter.setTargetTicks(650, Lifter.LIFTER_LEVEL.LOW.ticks - 900);

                    currentLifterState = Lifter.LIFTER_LEVEL.LOW;
                    currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;

                    intake.setCRServosPow(-0.8);

                    lifter.setTargetTicks(2500, Lifter.LIFTER_LEVEL.MID.ticks);
                })
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-45, -13, Math.toRadians(180)), Math.toRadians(180),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(25)
                )
                .splineToConstantHeading(new Vector2d(-57.7, -14), Math.toRadians(180))
                .build();
    }

    TrajectorySequence toMidFour(@NonNull TrajectorySequence toCone4) {
        return drive.trajectorySequenceBuilder(toCone4.end())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    intake.swingSetTargetTicks(250, Intake.INTAKE_STATE.INSIDE.ticks);
                    lifter.setTargetTicks(700, Lifter.LIFTER_LEVEL.MID.ticks);

                    currentLifterState = Lifter.LIFTER_LEVEL.MID;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                    intake.spinFor(2000, 800, 0.8);
                })
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-22.7, -23, Math.toRadians(113)), Math.toRadians(322),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(50, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(30)
                )

                .build();
    }


    TrajectorySequence toParkOne(@NonNull TrajectorySequence prevTraj) {
        return drive.trajectorySequenceBuilder(prevTraj.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, Intake.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                })

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50, -12, Math.toRadians(180)), Math.toRadians(180),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(60)
                )
                .lineTo(new Vector2d(-58, -13.5))

                .build();
    }

    TrajectorySequence toParkTwo(@NonNull TrajectorySequence prevTraj) {
        return drive.trajectorySequenceBuilder(prevTraj.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.DOWN.ticks);
                    intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.INSIDE.ticks);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                })

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-36.5, -15, Math.toRadians(90)), Math.toRadians(270),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(60)
                )

                .build();
    }

    TrajectorySequence toParkThree(@NonNull TrajectorySequence prevTraj) {
        return drive.trajectorySequenceBuilder(prevTraj.end())
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    automatism.goToState(currentLifterState, currentIntakeState, Lifter.LIFTER_LEVEL.DOWN, Intake.INTAKE_STATE.INSIDE);
                    currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
                    currentIntakeState = Intake.INTAKE_STATE.INSIDE;
                })

                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-11.5, -13.7, Math.toRadians(90)), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
                                )
                        ), new ProfileAccelerationConstraint(60)
                )

                .build();
    }
}
