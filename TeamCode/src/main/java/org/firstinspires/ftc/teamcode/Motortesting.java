import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Config
public class Motortesting extends LinearOpMode {

    public Lifter lifter;
    public double maxHeightTicks = 3100, minHeightTicks = -10;
    public double delta = 0.5;
    public double stand = 0.1;
    public static double kP = 11, kI = 0, kD = 0;
    public static double targetPosition = 2800;
    public double currentPosition;
    public static double maxPower = 0.6;
    double power, correction;


    @Override
    public void runOpMode() throws InterruptedException {

       lifter = new Lifter(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            //go to target position with given coefficients
            PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
            PIDFController controller = new PIDFController(coeffs);
            controller.setTargetPosition(targetPosition);

            if(gamepad2.a == true) {
                while (opModeIsActive()) {

                    currentPosition = lifter.lifterEncoder.getCurrentPosition();

                    correction = controller.update(currentPosition) / targetPosition;
                    power = Range.clip(correction, 0.1, maxPower);
                    lifter.setLifterPower(power);

                    telemetry.addData("ticksCurrent", currentPosition);
                    telemetry.addData("correction", correction);
                    telemetry.addData("power", power);
                    telemetry.update();


                    if (gamepad2.b == true) {
                        break;
                    }

                }
            }

            //telemetry for debugging
            telemetry.addData("ticks:", lifter.leftLifter.getCurrentPosition());
            telemetry.addData("trigger", gamepad2.right_trigger);
            telemetry.update();

            //lifter
            if(gamepad2.right_trigger != 0) {
                lifter.setLifterPower(Range.clip(gamepad2.right_trigger, 0, 0.5));
            }
            else if(gamepad2.left_trigger  != 0){
                lifter.setLifterPower(0);
            } else {
                lifter.setLifterPower(stand);
            }

            //limits! max = 3100, min = -10
            if(lifter.leftLifter.getCurrentPosition() >= maxHeightTicks) {
                lifter.setLifterPower(0.1);
            }

            if(lifter.leftLifter.getCurrentPosition() <= minHeightTicks) {
                lifter.setLifterPower(0.1);
            }
        }
    }
}
