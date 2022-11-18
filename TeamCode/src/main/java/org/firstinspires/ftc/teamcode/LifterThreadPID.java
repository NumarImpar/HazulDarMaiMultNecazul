//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import android.os.SystemClock;
//
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.roadrunner.util.ControllerInput;
//
//public class LifterThreadPID implements Runnable {
//
//    public double maxHeightTicks = 3100, minHeightTicks = -10;
//    public double delta = 0.5;
//    public double stand = 0.0;
//    public static double kP = 11, kI = 0, kD = 0;
//    public static double targetPosition = 1000;
//    public double currentPosition;
//    public static double maxPower = 0.6;
//    double power, correction;
//    public static double maxVel = 2120, maxAcc = 1500, maxJerk = 1500;
//
//
//    public static volatile boolean kill = false;
//    private long lastMillis = 0;
//    private volatile int currentTicks = 0;
//    private volatile int lastTicks = 0;
//
//    public static volatile boolean finishedUp = false;
//
//    @Override
//    public void run() {
//        while (!kill) {
//            //never run too fast
//            if (SystemClock.uptimeMillis() - lastMillis < 100) {
//                continue;
//            }
//
//            //set the last send time
//            lastMillis = SystemClock.uptimeMillis();
//
//            //pid-ul nostru
//
//            PIDCoefficients coeffs = new PIDCoefficients(0.1, 0, 0);
//            PIDFController controller = new PIDFController(coeffs);
//            controller.setTargetPosition(100);
//
//            if(finishedUp) {
//                currentPosition = LifterTest.lifter.getCurrentPosition();
//
//                correction = controller.update(currentPosition) / (targetPosition - 100);
//                power = Range.clip(correction, -0.1, 0.0);
//                LifterTest.lifter.setLifterPower(power);
//
//                telemetry.addData("ticksCurrent", currentPosition);
//                telemetry.addData("correction", correction);
//                telemetry.addData("power", power);
//                telemetry.update();
//
//                if(currentPosition - targetPosition < 100){
//                    finishedUp = false;
//                }
//            }
//
//            coeffs = new PIDCoefficients(kP, kI, kD);
//            controller = new PIDFController(coeffs);
//            controller.setTargetPosition(targetPosition);
//
//            if(!finishedUp){
//
//                correction = controller.update(currentPosition) / targetPosition;
//                power = Range.clip(correction, 0.1, maxPower);
//                LifterTest.lifter.setLifterPower(power);
//
////                telemetry.addData("ticksCurrent", currentPosition);
////                telemetry.addData("correction", correction);
////                telemetry.addData("power", power);
////                telemetry.update();
//
//                if(targetPosition - currentPosition < 100){
//                    finishedUp = true;
//                }
//            }
//            //gata pid-ul
//
//            lastTicks = currentTicks;
//
//        }
//    }
//
//    public void setTicks(int ticks) {
//        currentTicks = ticks;
//    }
//}