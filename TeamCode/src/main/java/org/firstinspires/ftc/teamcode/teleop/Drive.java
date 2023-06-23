package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNou;
import org.firstinspires.ftc.teamcode.mechanisms.LifterEx;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Automatisms;


@TeleOp
public class Drive extends LinearOpMode {

    public SampleMecanumDrive drive;
    private ControllerInput controller1, controller2;

    public LifterEx lifter;
    public IntakeNou intake;

    public static volatile boolean here = false;

    private enum DRIVE_MODE{
        MANUAL, //driver controlls everything
        AUTO,   //uses automatisms
        HYBRID
        //auto currently disabled
        //manual currently disabled
    }

    private DRIVE_MODE driveMode;

    /*private void handleManualControl(ControllerInput _controller){
     
    }*/


    /*private void handleAutomizedControl(ControllerInput _controller) {

    }*/

    private int deg = 10;
    private boolean lateralGrab = false;
    private double sliderPos = 1d;
    private final double kSlide = .0005;
    private void handleHybridControl(ControllerInput _controller) {
        if(_controller.options()){ // skip if switching player
            return;
        }

        // toggle claw
        // left trigger once is a new function in ControllerInput
        // if it doesnt work change it for something else
        if(_controller.leftTriggerOnce()){
            intake.clawToggle(0);
        }

        // go to high - lifter
        // previous high was at 2700 but its not really useful anymore
        if(_controller.triangleOnce()){
            lifter.setTargetTicks(0, 1800);
        }

        // go to mid - lifter
        // previous mid was 1800
        if(_controller.squareOnce()){
            lifter.setTargetTicks(0, 1000);
        }

        // go to down - lifter
        // init is at 0 but that position lets the claw hang dangerously close to the tiles
        if(_controller.crossOnce()){
            lifter.setTargetTicks(0, 200);
        }

        // toggle 180 - 10 deg intake
        if(_controller.circleOnce()){
            if(deg < 100){
                deg = 180;
            } else {
                deg = 10;
            }

            intake.setTargetDeg(deg);
        }

        // note that after a lateral grab one can rotate the intake to stack cones
        // on deg > 100 lateral grab clears automatically
        if(deg > 100){
            intake.rotateClawToAngle(180);
            lateralGrab = false;
        } else {
            if(!lateralGrab){
                intake.rotateClawToAngle(0);
            }
        }

        // lateral grab - might work on fallen down cones
        if(_controller.leftBumperOnce()){
            if(! lateralGrab){
                if(deg < 50) { // lateral grab has no purpose in the air, so put this to prevent accidental presses
                    intake.rotateClawToAngle(90);
                    lateralGrab = true;
                }
            } else {
                lateralGrab = false;
                intake.rotateClawToAngle(0);
            }
        }

        // extend slider slowly with right_bumper_x
        if(_controller.right_bumper_x > .3){
            sliderPos -= right_bumper_x * kSlide;
            sliderPos = Range.clip(sliderPos, .4, 1d);
            intake.extendSlider(0, sliderPos);
        }

        // extend slider quickly - max length
        if(_controller.dpadUpOnce()){
            sliderPos = .4;
            intake.extendSlider(0, sliderPos);
        }

        // retract slider quickly - min legth
        if(_controller.dpadDownOnce()){
            sliderPos = 1d;
            intake.extendSlider(0, sliderPos);
        }
    }

    private void handleDriving(ControllerInput _controller) {
        if (_controller.options()) { // skip if switching player
            return;
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -_controller.left_stick_y,
                        -_controller.left_stick_x,
                        -_controller.right_stick_x
                )
        );

        drive.update();
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

    private void loopThroughDrivemodes(){
        return;

        if(driveMode == DRIVE_MODE.HYBRID){
            driveMode = DRIVE_MODE.AUTO;
        } else if (driveMode == DRIVE_MODE.AUTO){
            driveMode = DRIVE_MODE.MANUAL;
        } else {
            driveMode = DRIVE_MODE.HYBRID;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // ------------- INIT ----------------
        driveMode = DRIVE_MODE.HYBRID;

        drive = new SampleMecanumDrive(hardwareMap);
        lifter = new LifterEx(hardwareMap);
        intake = new IntakeNou(hardwareMap, lifter);

        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2, .5, .5);

        telemetry.addLine("mechanisms initialized. press START");
        telemetry.update();
        
        // -------------- START --------------
        waitForStart(); 

        intake.startIntakeThread();
        lifter.startLifterThread();
        intake.endClawProcessesForcibly();

        // claw will push too much into the floor otherwise
        // no need to wait until these finish since newer targets take priority in the run() functions
        lifter.setTargetTicks(0, 200);
        intake.setTargetDeg(0, 15);
        
        // ------------- DRIVE --------------
        while(opModeIsActive()){
            controller1.update();
            controller2.update();

            telemetry.addData("mode", driveMode);
            telemetry.update();

            // controller 2
            switch(driveMode){
                case MANUAL: {
                    break;
                }
                case AUTO: { 
                    break;
                }
                case HYBRID: {
                    handleHybridControl(controller2);
                    break;
                }
            }

            // change drive modes
            if(controller2.shareOnce()){
                loopThroughDrivemodes();
            }

            // controller 1
            if (controller1.rightBumper()) {
                handleDrivingSlowed(controller1);
            } else {
                handleDriving(controller1);
            }

            if(lifter.getCurrentPosition() > 3000){
                lifter.killLifterThread();
                telemetry.setAutoClear(false);
                telemetry.addLine("Lifter went too high unexpectedly. Thread killed. STOP and REINITIALIZE");
                telemetry.update();
            }
        }

        // --------------- STOP ----------------
        intake.endClawProcessesForcibly();
        lifter.killLifterThread();
        intake.killIntakeThread();
    }
}

