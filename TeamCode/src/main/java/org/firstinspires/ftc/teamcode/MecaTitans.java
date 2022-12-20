package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;


public class MecaTitans {
    private final Telemetry telemetry;
    private DcMotor lift_right, lift_left;
    private SampleMecanumDrive drive;

    private Servo link1, link2;
    private CRServo intake1, intake2;


    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.004, 0, 0.0002);

    // Copy your feedforward gains here
    public static double kV = 0.000427;
    public static double kA = 0.000366;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    // Our velocity controller
    //private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);


    public MecaTitans(HardwareMap hardwareMap, Telemetry telemetry) {
        //public Mechanism(HardwareMap hardwareMap, GAMEPAD gamepad1, Telemetry telemetry){

        this.telemetry = telemetry;

        initMechanism(hardwareMap);
    }

    private void initMechanism(HardwareMap hardwareMap) {

        drive = new SampleMecanumDrive(hardwareMap);

        lift_left = hardwareMap.get(DcMotorEx.class, "lifterLeft");
        lift_right = hardwareMap.get(DcMotorEx.class, "lifterRight");

        lift_right.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_left.setDirection(DcMotorSimple.Direction.FORWARD);

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


        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    public void mechanism(Gamepad gamepad2) {
        intake(gamepad2);
        lift(gamepad2);
        stopRight();
        swing(gamepad2);

        telemetry.addData("lift: ", lift_left.getCurrentPosition());
        telemetry.update();


//        telemetry.addData("velocity: ", );


    }


    private void lift(Gamepad gamepad2) {
        if (gamepad2.a) {
            lift_left.setTargetPosition(50);
            lift_left.setPower(-1);
            lift_right.setPower(-1);
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (gamepad2.y) {
            lift_left.setTargetPosition(2650);
            lift_left.setPower(-1);
            lift_right.setPower(-1);
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad2.x) {
            lift_left.setTargetPosition(500);
            lift_left.setPower(-1);
            lift_right.setPower(-1);
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        } else if (gamepad2.dpad_down) {
            lift_left.setTargetPosition(lift_left.getCurrentPosition() - 40);
            lift_left.setPower(1);
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        } else if (gamepad2.dpad_up) {
            lift_left.setTargetPosition(lift_left.getCurrentPosition() + 40);
            lift_left.setPower(-1);
            lift_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

    public void stopRight(){
        if (!lift_left.isBusy()){
            lift_right.setPower(0);
        }
    }

    private void intake(Gamepad gamepad2) {
        if (gamepad2.right_trigger > 0.3) {
            intake2.setPower(0.5);
            intake1.setPower(0.5);
        } else if (gamepad2.left_trigger > 0.3) {
            intake1.setPower(-0.3);
            intake2.setPower(-0.3);
        } else {
            intake1.setPower(0);
            intake2.setPower(0);
        }
    }



    //TODO:swing

    private void swing(Gamepad gamepad2){
        if(gamepad2.right_bumper){
            link2.setPosition(1);
            link1.setPosition(0);
        }
        else if(gamepad2.left_bumper){
            link2.setPosition(1);
            link1.setPosition(0);
        }
    }


}
