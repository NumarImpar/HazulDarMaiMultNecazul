import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.util.ControllerInput;
import org.firstinspires.ftc.teamcode.*;

//jUST A TEST FILE: not to be taken in consideration.
@TeleOp
@Config
public class LifterTest extends LinearOpMode {

    public Lifter lifter;
    public ControllerInput controller1, controller2;

    public double maxHeightTicks = 3100, minHeightTicks = -10;
    public double delta = 0.5;
    public double stand = 0.0;
    public static double kP = 11, kI = 0, kD = 0;
    public static double targetPosition = 1000;
    public double currentPosition;
    public static double maxPower = 0.6;
    double power, correction;
    public static double maxVel = 2120, maxAcc = 1500, maxJerk = 1500;

    @Override
    public void runOpMode() throws InterruptedException {

        lifter = new Lifter(hardwareMap, telemetry);
        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
/*
        lifterThreadPID = new LifterThreadPID();
*/
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();
        Thread lifterThread = new Thread(lifter);
        lifterThread.start();

        while (opModeIsActive()) {

            controller2.update();
	    double target = 0;

            if(controller2.AOnce()){
                lifter.setTargetTicks(2500);
		target = 2500;
            }

            if(controller2.BOnce()){
                lifter.setTargetTicks(50);
		target = 50;
            }
           
	}
        lifterThread.interrupt();
    }
}

