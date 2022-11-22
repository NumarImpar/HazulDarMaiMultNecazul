
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.roadrunner.util.ControllerInput;

@Config
@TeleOp
public class LifterVelocityTuner extends LinearOpMode {

    Lifter lifter;
    ControllerInput controller1;
    double maxVel, maxAcc, maxJerk;

    public static double MAX_POWER = 0.6;
    public static double MAX_TIME = 1000; // ms

    @Override
    public void runOpMode() throws InterruptedException {
        lifter = new Lifter(hardwareMap, telemetry);
        controller1 = new ControllerInput(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        double prevTime = 0.0;
        double prevVel = 0.0;
        double prevAcc = 0.0;

        lifter.setLifterPower(MAX_POWER);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < MAX_TIME && opModeIsActive()) {

            double currentTime = timer.seconds();
            double deltaTime = currentTime - prevTime;

            double currentVel = lifter.getCorrectedVelocity();
            double currentAcc = (currentVel - prevVel) / deltaTime;
            double currentJerk = (currentAcc - prevAcc) / deltaTime;

            maxVel = Math.max(maxVel, currentVel);
            if (timer.milliseconds() > 50) {
                maxAcc = Math.max(maxAcc, currentAcc);
                maxJerk = Math.max(maxJerk, currentJerk);
            }

            prevVel = currentVel;
            prevAcc = currentAcc;
            prevTime = currentTime;

            telemetry.addData("Position", lifter.getCurrentPosition());
            telemetry.addData("Delta time", deltaTime);
            telemetry.addData("Current Velocity", currentVel);
            telemetry.addData("Current Acceleration", currentAcc);
            telemetry.addData("Current Jerk", currentJerk);
            telemetry.update();
        }
        lifter.setLifterPower(0.0);

        telemetry.log().clear();
        telemetry.addData("Maximum Velocity", maxVel);
        telemetry.addData("Maximum Acceleration", maxAcc);
        telemetry.addData("Maximum Jerk", maxJerk);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) idle();
    }
}
