package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.ControllerInput;

@TeleOp
public class Drive extends LinearOpMode {
    Intake intake;
    Lifter lifter;
    SampleMecanumDrive drive;
    DcMotorEx LF, LB, RF, RB;
    ControllerInput controller1, controller2;

    public static int forward = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        LF = drive.LF;
        LB = drive.LB;
        RF = drive.RF;
        RB = drive.RB;

        waitForStart();

        Thread lifterThread = new Thread(lifter);
        lifterThread.start();

        while (opModeIsActive()) {

            controller1.update();
            controller2.update();

            //right bumper - high
            if(controller2.rightBumperOnce()){
                lifter.setTargetTicks(0,2600);
            }

            //triunghi - poz intermediare - mid
            if(controller2.YOnce() && !gamepad2.start){
                lifter.setTargetTicks(0,1700);
            }

            //circle - cone out
            if(controller2.BOnce() && !gamepad2.start){
                intake.spinReverseForMs(0,800);
            }

            //x - cone in
            if (controller2.AOnce()&& !gamepad2.start) {
                intake.spinForwardForMs(0,800);
            }


            //square - intake - pozitie de colectare
            //explicatie mai jos
            /*if left servo is at 0 then go at 0.4
            //if left servo is at 1 then go at 0.7*/
            if(controller2.XOnce() && !gamepad2.start) {
                if (intake.leftServoIntake.getPosition() == 0) {
                    intake.moveIntake(0.35);
                } else if (intake.leftServoIntake.getPosition() == 1) {
                    intake.moveIntake(0.65);
                }
            }

            //arrow down - intake outside
            if(controller2.dpadDownOnce()){
                    intake.moveIntake(1);
            }

            //arrow up - intake inside
            if(controller2.dpadUpOnce()){
                    intake.moveIntake(0);
            }

            //left bumper - lifter down
            if(controller2.leftBumper()){
                lifter.setTargetTicks(0,50);
            }

            //automatism
            if(controller1.leftBumperOnce()){
                lifterHighIntakeOutCone();
            }

            if(controller1.AOnce()){
                collectFromInside();
            }

            if (controller1.rightBumper()) {
                handleDrivingSlowed();
            } else handleDriving();
        }

        lifterThread.interrupt();

    }

    private void handleDriving() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * 0.7,
                        -gamepad1.left_stick_x * 0.7,
                        gamepad1.right_stick_x * 0.7
                        //forward version has - at right_stick
                )
        );
    }

    private void handleDrivingSlowed(){
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * 0.35,
                        -gamepad1.left_stick_x * 0.35,
                        gamepad1.right_stick_x * 0.35
                )
        );
    }

    //automatism - lifter high - intake out (if not already) - get cone out
    private void lifterHighIntakeOutCone(){
        lifter.setTargetTicks(0,2600);
        if(intake.leftServoIntake.getPosition()==0){
            intake.moveIntakeArm(200, 1);
        }
        //intake.spinReverseForMs(6000, 800);
    }

    //automatism - lifter down (from high) - intake in
    private void lifterDownIntakeIn(){
        lifter.setTargetTicks(800, 50);
        if(intake.leftServoIntake.getPosition() != 0){
            intake.moveIntakeArm(0, 0);
        }
    }

    //automatism - lifter semi down - collect from front
    private void collectFromInside(){
        lifter.setTargetTicks(0, 1000);
        if(intake.leftServoIntake.getPosition() != 0){
            intake.moveIntakeArm(500, 0.3);
        }
    }

    //automatism - lifter down - get cone in
    private void lifterDownConeIn(){
        lifter.setTargetTicks(0,50);
        if(Math.abs(lifter.lifterEncoder.getCurrentPosition()-50)<30){
            intake.spinForwardForMs(1000,800);
        }
    }

}
