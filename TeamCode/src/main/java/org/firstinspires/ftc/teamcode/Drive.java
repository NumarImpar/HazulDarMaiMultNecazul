import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

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
        boolean alrup = false;
        while (opModeIsActive()) {

            controller1.update();
            controller2.update();

	    if (controller2.YOnce()){
	        intake.moveIntake(0);
		//intake.switchState(true);
		alrup = false;
	    }
	    if (controller2.XOnce()){
		    if (alrup){
		        intake.moveIntake(0.7);
			alrup = false;
		    } else {
		        intake.moveIntake(0.5);
			alrup = true;
		    }
            }
            if (controller2.AOnce()) {
                lifter.setTargetTicks(2500);
            }

            if (controller2.BOnce()) {
                lifter.setTargetTicks(50);
            }

            //get cone in
            if (controller2.leftBumperOnce()) {
                intake.spinForwardForMs(300);
            }

            //get cone out
            if (controller2.rightBumperOnce()){
                intake.spinReverseForMs(300);
            }

            handleDriving();
            if (controller1.leftBumper()) {
                handleDrivingSlowed();
            }
        }

        lifterThread.interrupt();

    }

    private void handleDriving() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
    }

    private void handleDrivingSlowed(){
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * 0.6,
                        -gamepad1.left_stick_x * 0.6,
                        -gamepad1.right_stick_x * 0.6
                )
        );
    }

}

