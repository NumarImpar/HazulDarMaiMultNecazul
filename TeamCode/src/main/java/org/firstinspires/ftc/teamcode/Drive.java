package org.firstinspires.ftc.teamcode;

import java.util.function.Consumer;
import java.util.List;
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
    
    public enum LifterStates {
	HIGH, DOWN, MID, LOW
    }
    public enum DrivingStates {
	PARKED, DRIVING_FAST, DRIVING_SLOW
    }
    public enum IntakeStates {
	INSIDE, OUTSIDE, GET_CONE, OUTSIDE_DOWN
    }
    public volatile boolean actualSpinState = false;
    public volatile DrivingStates actualDrivingState = DrivingStates.PARKED;
    public volatile IntakeStates actualIntakeState = IntakeStates.INSIDE;
    public volatile LifterStates actualLifterState = LifterStates.DOWN;
    public static int forward = 1;
    public static volatile boolean virginIntake = false; // mitigation for hitting lifter pane
    public static volatile int virginThreshold = 300;

    public long lifterWait = 0;
    Consumer<LifterStates> goToLifterState = (s) -> {
        if (s == LifterStates.HIGH){
	    lifter.setTargetTicks(lifterWait, 2650);
	} else if (s == LifterStates.MID){
	    lifter.setTargetTicks(lifterWait, 1700);
	} else if (s == LifterStates.LOW) {
	    lifter.setTargetTicks(lifterWait, 1000);
	} else if (s == LifterStates.DOWN){
	    lifter.setTargetTicks(lifterWait, 0);
	}
	actualLifterState = s;
	return;
    };

    public long intakeWait = 0;
    Consumer<IntakeStates> goToIntakeState = (s) -> {
        if (s == IntakeStates.INSIDE){
	    intake.moveIntakeArm(intakeWait, 0);
	} else if (s == IntakeStates.OUTSIDE){
	    intake.moveIntakeArm(intakeWait, 1);
	} else if (s == IntakeStates.GET_CONE){
	    intake.moveIntakeArm(intakeWait, 0.3);
	}
    };

    public void goToState(LifterStates _lifterstate, IntakeStates _intakestate){
	if (_intakestate == IntakeStates.INSIDE && _lifterstate == LifterStates.DOWN){
	    if (actualIntakeState != IntakeStates.OUTSIDE){
	        intakeWait = 200;
		intake.moveIntakeArm(0, 0.4);
	    } else if (actualLifterState == LifterStates.HIGH){
	    intakeWait = 930;
	    virginThreshold = 200;
	    } else {
	        virginThreshold = 100;
	    }
	} else if (_intakestate == IntakeStates.OUTSIDE && _lifterstate == LifterStates.HIGH){
	    intakeWait = 130;
	} else if (_intakestate == IntakeStates.GET_CONE && _lifterstate == LifterStates.MID){
	    intakeWait = 500; 
	} else if (_intakestate == IntakeStates.OUTSIDE && _lifterstate == LifterStates.MID){
	    lifterWait = 500;
	} 
	
	goToIntakeState.accept(_intakestate);
	goToLifterState.accept(_lifterstate);
	lifterWait = 0;
	intakeWait = 0;
    }

    public void getConeIn(){
        if(Math.abs(lifter.lifterEncoder.getCurrentPosition()-50)<30){
            intake.spinForwardForMs(1000,800);
        }
    }
    
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
	Thread intakeThread = intake.moveIntakeArm(0, 0);
        lifterThread.start();

	Thread drivingStateWatchdog = new Thread(() -> {
	    while(this.opModeIsActive()){
	        if (!drive.isBusy()){
		    actualDrivingState = DrivingStates.PARKED;
		} else {
		    List<Double> velocities = drive.getWheelVelocities();
		    int counter = 0;
		    for (Double vel : velocities){
		        counter += (vel >= 15)?(1):((vel <= -15)?(1):(0)); 
		    }
		    if (counter >= 2){
		        actualDrivingState = DrivingStates.DRIVING_FAST;
		    } else {
		        actualDrivingState = DrivingStates.DRIVING_SLOW;
		    }
		}
	    }
	    return;
	});

	drivingStateWatchdog.start();


        while (opModeIsActive()) {

            controller1.update();
            controller2.update();

            //right bumper - high
            if(controller2.rightBumperOnce()){
                goToState(LifterStates.HIGH, IntakeStates.OUTSIDE);
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
                    intake.moveIntakeArm(0, 0);
            }

            //arrow up - intake inside
            if(controller2.dpadUpOnce()){
                    intake.moveIntakeArm(0, 1);
            }

            //left bumper - lifter down
            if(controller2.leftBumper()){
                lifter.setTargetTicks(0,50);
            }

            if(controller2.left_trigger > 20){
                goToState(LifterStates.DOWN, IntakeStates.INSIDE);
            }

            if (controller1.rightBumper()) {
                handleDrivingSlowed();
            } else handleDriving();
        }

	intake.kill = true;
        lifterThread.interrupt();
	intakeThread.interrupt();
        drivingStateWatchdog.interrupt();
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
	actualLifterState = LifterStates.DOWN;
        if(Math.abs(lifter.lifterEncoder.getCurrentPosition()-50)<30){
            intake.spinForwardForMs(1000,800);
        }
    }


}
