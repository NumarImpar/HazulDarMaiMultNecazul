package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Automatisms;

/* A NU SE FOLOSI IN MECI LMAO*/


@TeleOp
public class DriveMEME extends LinearOpMode {

    public SampleMecanumDrive drive;
    private ControllerInput controller1, controller2;
    private Automatisms automatism;

    public Lifter lifter;
    public Intake intake;

    public Lifter.LIFTER_LEVEL currentLifterState;
    public Intake.INTAKE_STATE currentIntakeState;

    protected Thread intakeThread, lifterThread;

    public static volatile boolean here = false;

    private enum DRIVE_MODE {
        MANUAL, //driver controlls everything
        AUTO    //uses automatisms
    }

    private DRIVE_MODE driveMode;

    public boolean paConIsInside = true;


    //NU FOLOSIM ASTA!! STAM MEREU PE AUTO!!
    private void handleManualControl(ControllerInput _controller) {
//        if (_controller.right_stick_y == 0){
//            lifter.setManualPower(0);
//            lifter.manual = false;
//        }

        // right stick y: lifter manual mode
        if (_controller.right_stick_y != 0) {
            lifter.manual = true;
            lifter.setManualPower(-_controller.right_stick_y * 0.4);
        }

        //lifter high
        if (_controller.rightBumper()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.HIGH.ticks);
            currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
        }

        // lifter down on left bumper
        if (_controller.leftBumper()) {
            lifter.setTargetTicks(0, 50);
        }

        //intake out (swing)
        if (_controller.dpadUpOnce()) {
            intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.OUTSIDE.ticks);
        }

        //intake in (swing)
        if (_controller.dpadDownOnce()) {
            intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.INSIDE.ticks);
        }

        //cone out
        if (_controller.left_trigger > 0.6) {
            intake.spinFor(0, 800, -0.6);
        }

        //cone in
        if (_controller.right_trigger > 0.6) {
            intake.spinFor(0, 800, 0.6);
        }


        //lifter mid
        if (_controller.triangle()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.MID.ticks);
        }

        //lifter low or somethin
        if (_controller.square()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.LOW.ticks);
        }
    }


    private void handleAutomizedControl(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }

        //intake out (swing)
        if (_controller.dpadUpOnce()) {
            intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.OUTSIDE.ticks);
            currentIntakeState = Intake.INTAKE_STATE.OUTSIDE;
        }

        //intake in (swing)
        if (_controller.dpadDownOnce()) {
            intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.INSIDE.ticks);
            currentIntakeState = Intake.INTAKE_STATE.INSIDE;
        }

        //low
        if (_controller.circleOnce()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.LOW.ticks);
            currentLifterState = Lifter.LIFTER_LEVEL.LOW;
        }

        //down cu intake unde era
        if (_controller.leftBumperOnce()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.DOWN.ticks);
            currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        }

        //poz de colectare
        if (_controller.squareOnce()) {
            if (currentIntakeState == Intake.INTAKE_STATE.OUTSIDE) {
                intake.swingSetTargetTicks(0, 1100);
            } else {
                intake.swingSetTargetTicks(0, 30);
            }
        }

        if (_controller.crossOnce()) {
            lifter.setTargetTicks(0, 800);
            currentLifterState = Lifter.LIFTER_LEVEL.LOW;
        }

        //cone in/out while pressing
        if (_controller.left_trigger > 0.5) {
            intake.setCRServosPow(0.8);
        } else if (_controller.right_trigger > 0.5) {
            intake.setCRServosPow(-0.8);
        } else {
            intake.setCRServosPow(0.0);
        }

        //high intake unde era
        if (_controller.rightBumperOnce()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.HIGH.ticks);
            currentLifterState = Lifter.LIFTER_LEVEL.HIGH;
        }

        if (_controller.triangleOnce()) {
            lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.MID.ticks);
            currentLifterState = Lifter.LIFTER_LEVEL.MID;
        }
    }

    private void handleDriving(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }

        drive.update();
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addData("x", drive.getPoseEstimate().getX());
        telemetry.addData("y", drive.getPoseEstimate().getY());
        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.update();

    // Create a vector from the gamepad x/y inputs
    // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

    // Pass in the rotated input + right stick value for rotation
    // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
    }

    private void handleDrivingSlowed(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }

        drive.setWeightedDrivePower(new Pose2d(
                -_controller.left_stick_y * 0.35,
                -_controller.left_stick_x * 0.35,
                -_controller.right_stick_x * 0.35
        ));

        drive.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        driveMode = DRIVE_MODE.AUTO;

        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        automatism = new Automatisms(lifter, intake);

        currentLifterState = Lifter.LIFTER_LEVEL.DOWN;
        currentIntakeState = Intake.INTAKE_STATE.INIT;

        intake.spinFor(0, 50, 0.1); //bug

        waitForStart();

        intakeThread = new Thread(intake);
        lifterThread = new Thread(lifter);

        lifterThread.start();
        intakeThread.start();

        while (opModeIsActive()) {

            controller1.update();
            controller2.update();

//            telemetry.addData("intakePos", intake.getSwingPos());
//            telemetry.addData("MODE:", driveMode);
//            telemetry.addData("here", here);
//            telemetry.update();

            //controller 2 - mechanisms n stuff
            switch (driveMode) {
                case MANUAL: {
                    handleManualControl(controller2);

                    if (controller2.shareOnce()) {
                        driveMode = DRIVE_MODE.AUTO;
                    }

                    break;
                }
                case AUTO: { //rn stam doar pe cazul auto
                    handleAutomizedControl(controller2);

//                    if(controller2.shareOnce()){
//                        driveMode = DRIVE_MODE.MANUAL;
//                    }

                    break;
                }
            }

            // controller 1 - chassis + ia lasa con
            if (controller1.rightBumper()) {
                handleDrivingSlowed(controller1);
            } else {
                handleDriving(controller1);
            }

            if (controller1.left_trigger > 0.2) {
                intake.setCRServosPow(-1);
            } else if (controller1.right_trigger > 0.2) {
                intake.setCRServosPow(1);
            } else {
                intake.setCRServosPow(0);
            }
        }

        lifter.kill = true;
        intake.kill = true;

    }
}

