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

            if(controller2.AOnce()){
                lifter.setTargetTicks(1500);
            }

            if(controller2.BOnce()){
                lifter.setTargetTicks(50);
            }
/*
            if (controller2.YOnce()) {
//                //go to target position with given coefficients
//                PIDCoefficients coeffs = new PIDCoefficients(0.1, 0, 0);
//                PIDFController controller = new PIDFController(coeffs);
//                controller.setTargetPosition(100);
//
//                while (opModeIsActive()) {
//                    controller2.update();
//                    currentPosition = lifter.getCurrentPosition();
//
//                    correction = controller.update(currentPosition) / (targetPosition - 100);
//                    power = Range.clip(correction, -0.1, 0.0);
//                    lifter.setLifterPower(power);
//
//                    telemetry.addData("ticksCurrent", currentPosition);
//                    telemetry.addData("correction", correction);
//                    telemetry.addData("power", power);
//                    telemetry.update();


            }
        }

//                //go to target position with given coefficients
//                PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
//                PIDFController controller = new PIDFController(coeffs);
//                controller.setTargetPosition(targetPosition);
//
//                while (opModeIsActive()) {
//                    controller2.update();
//                    currentPosition = lifter.getCurrentPosition();
//
//                    correction = controller.update(currentPosition) / targetPosition;
//                    power = Range.clip(correction, 0.1, maxPower);
//                    lifter.setLifterPower(power);
//
//                    telemetry.addData("ticksCurrent", currentPosition);
//                    telemetry.addData("correction", correction);
//                    telemetry.addData("power", power);
//                    telemetry.update();

//                MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
//                        new MotionState(lifter.getCurrentPosition(), 0, 0),
//                        new MotionState(targetPosition, 0, 0),
//                        maxVel,
//                        maxAcc,
//                        maxJerk
//                );
//                PIDCoefficients pidCoefficients = new PIDCoefficients(kP, kI, kD);
//                PIDFController controller = new PIDFController(pidCoefficients);
//                ElapsedTime timer = new ElapsedTime();
//                timer.reset();
//
//                telemetry.addData("duration", profile.duration());
//                telemetry.update();
//
//                while (opModeIsActive()) {
//                    controller2.update();
//                    MotionState state = profile.get(timer.seconds());
//
//                    controller.setTargetPosition(state.getX());
//                    controller.setTargetVelocity(state.getV());
//                    controller.setTargetAcceleration(state.getA());
//
//                    telemetry.addData("getX", state.getX());
//                    telemetry.addData("getV", state.getV());
//                    telemetry.addData("getA", state.getA());
//
//
//                    double currentPosition = lifter.getCurrentPosition();
//                    double currentVelocity = lifter.getCorrectedVelocity();
//
//                    double correction = controller.update(currentPosition, currentVelocity);
//                    lifter.setLifterPower(Range.clip(correction, 0, 0.6));
//                    telemetry.addData("Correction", correction);
//                    telemetry.update();
//
//                    if (Math.abs(currentPosition - targetPosition) < 100.0 || controller2.BOnce()) {
//                        break;
//                    }
//                }
//                //lifter.setLifterPower(0.1);
//                telemetry.log().clear();
//                telemetry.update();
//                //maintain pid
//                PIDCoefficients coeffs = new PIDCoefficients(11, 0, 0);
//                controller = new PIDFController(coeffs);
//                controller.setTargetPosition(targetPosition);
//                while (opModeIsActive() && !controller2.BOnce()) {
//                    currentPosition = lifter.getCurrentPosition();
//
//                    correction = controller.update(currentPosition) / targetPosition;
//                    power = Range.clip(correction, 0.0, 0.6);
//                    lifter.setLifterPower(power);
//
//                    telemetry.addData("position", currentPosition);
//                    telemetry.update();
//                }
//            }


            //mod manual slash automatizat
            if (gamepad2.options && gamepad2.x) {
                lifterMode = Lifter.MODE.MANUAL;
            }

            //telemetry for debugging
//            telemetry.addData("ticks:", lifter.leftLifter.getCurrentPosition());
//            telemetry.addData("trigger", controller2.right_trigger);
//            telemetry.update();

            //lifter manual controls
            if (lifterMode.equals(Lifter.MODE.MANUAL)) {
                if (controller2.right_trigger != 0) {
                    lifter.setLifterPower(Range.clip(controller2.right_trigger, 0, 0.5));
                } else if (controller2.left_trigger != 0) {
                    lifter.setLifterPower(-1 * Range.clip(controller2.left_trigger, 0, 0.5));
                } else {
                    lifter.setLifterPower(stand);
                }

                //limits! max = 3100, min = -10
                if (lifter.leftLifter.getCurrentPosition() >= maxHeightTicks) {
                    lifter.setLifterPower(0);
                }

                if (lifter.leftLifter.getCurrentPosition() <= minHeightTicks) {
                    lifter.setLifterPower(0);
                }
            }
            telemetry.addData("ticksCurrent", lifter.getCurrentPosition());
            telemetry.addData("correction", correction);
            telemetry.addData("power", power);
            telemetry.update();*/
        }

        lifterThread.interrupt();
    }
}

