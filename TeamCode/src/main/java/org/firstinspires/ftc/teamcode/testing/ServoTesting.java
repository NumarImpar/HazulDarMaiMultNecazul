package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp
public class ServoTesting extends LinearOpMode {

    ControllerInput controller;
    Intake intake;
    Thread intakeThread;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        controller = new ControllerInput(gamepad1);
        intakeThread = new Thread(intake);
        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        //intakeThread.start();

        //intake.paConInterpolateThread(0, 0.96, 0.45, 2000, 40);

        //sleep(5000);

        while(opModeIsActive()){
            controller.update();

//            drive.RF.setPower(0.3);
//            sleep(2000);
//            drive.RF.setPower(0);
//            sleep(2000);
//
//            drive.LF.setPower(0.3);
//            sleep(2000);
//            drive.LF.setPower(0);
//            sleep(2000);
//
//            drive.RB.setPower(0.3);
//            sleep(2000);
//            drive.RB.setPower(0);
//            sleep(2000);
//
//
//            drive.LB.setPower(0.3);
//            sleep(2000);
//            drive.LB.setPower(0);
//            sleep(6000);

            drive.RF.setPower(0.3);
            drive.LF.setPower(0.3);
            drive.RB.setPower(0.3);
            drive.LB.setPower(0.3);

            sleep(3000);

            drive.RF.setPower(0);
            drive.LF.setPower(0);
            drive.RB.setPower(0);
            drive.LB.setPower(0);

            sleep(1000);

            drive.RF.setPower(-0.3);
            drive.LF.setPower(-0.3);
            drive.RB.setPower(-0.3);
            drive.LB.setPower(-0.3);

            sleep(3000);

            drive.RF.setPower(0);
            drive.LF.setPower(0);
            drive.RB.setPower(0);
            drive.LB.setPower(0);

            sleep(1000);

        }
        //intake.kill = true;
    }
}
