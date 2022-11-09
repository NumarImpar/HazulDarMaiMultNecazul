import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Motortesting extends LinearOpMode {

    public Lifter lifter;
    public int maxHeightTicks = 3100;
    public int minHeightTicks = -10;
    public double delta = 0.5;
    public double stand = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {

       lifter = new Lifter(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            //telemetry for debugging
            telemetry.addData("ticks:", lifter.leftLifter.getCurrentPosition());
            telemetry.addData("trigger", gamepad1.right_trigger);
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
            if(lifter.leftLifter.getCurrentPosition() > maxHeightTicks && gamepad2.right_trigger != 0) {
                lifter.setLifterPower(0);
            }

            if(lifter.leftLifter.getCurrentPosition() <= minHeightTicks && gamepad2.left_trigger != 0) {
                lifter.setLifterPower(0);
            }
        }
    }
}
