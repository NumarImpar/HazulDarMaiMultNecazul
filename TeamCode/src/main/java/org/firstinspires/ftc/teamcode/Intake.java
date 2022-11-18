import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl.*;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake{
    public CRServo leftCRServoIntake;
    public CRServo rightCRServoIntake;
    public ElapsedTime timer;
    public ServoImplEx leftServoIntake;
    public ServoImplEx rightServoIntake;

    public Intake(HardwareMap _hardwareMap){
        leftCRServoIntake = _hardwareMap.get(CRServo.class, "leftCRServoIntake");
        rightCRServoIntake = _hardwareMap.get(CRServo.class, "rightCRServoIntake");
	leftServoIntake = _hardwareMap.get(ServoImplEx.class, "leftServoSwing");
        rightServoIntake = _hardwareMap.get(ServoImplEx.class, "rightServoSwing");

	leftServoIntake.setPwmRange(new PwmRange(500, 2500));
	rightServoIntake.setPwmRange(new PwmRange(500, 2500));
        leftCRServoIntake.setDirection(CRServo.Direction.REVERSE);
        rightCRServoIntake.setDirection(CRServo.Direction.FORWARD);
    }

    public void setCRServosIntakePower(double _power1, double _power2){
        leftCRServoIntake.setPower(_power1);
        rightCRServoIntake.setPower(_power2);
    }

    public volatile boolean running = false;

    public void startSpin() {
        setCRServosIntakePower(0.3, 0.3);
        running = true;
    }

    public void stopSpin() {
        setCRServosIntakePower(0.0, 0.0);
        running = false;
    }

    public void startReverseSpin(){
        setCRServosIntakePower(-0.3, -0.3);
        running = true;
    }

    public Thread spinForwardForMs(long ms) {
        if(ms <= 0) return new Thread();
        Thread t = new Thread(() -> {
            startSpin();
            try {
                Thread.sleep(ms);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopSpin();
        });
	t.start();
	return t;
    }

    public Thread spinReverseForMs(long ms) {
        if(ms <= 0) return new Thread();
        Thread t = new Thread(() -> {
            startReverseSpin();
            try {
                Thread.sleep(ms);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopSpin();
        });
	t.start();
	return t;
    }

    public void moveIntake(double pos){
        leftServoIntake.setPosition(Range.clip(pos, -1, 1));
	rightServoIntake.setPosition(Range.clip(1 - pos, -1, 1));
    /*
	    leftServoIntake.setPower(pos);
	    rightServoIntake.setPower(-pos);
    }

    public void switchState(boolean state) {
        Thread t = new Thread(() -> {
	    moveIntake((state)?(0.3d):(-0.4d));
	   try{Thread.sleep(1000);} catch (InterruptedException e ) {;;} 
	   moveIntake((state)?(0.04d):(-0.6d));
	});
	t.start();*/
    }


 //   public volatile int f = 2;

//    //evident.
//    public void getConeIn() {
//        timer = new ElapsedTime();
//        timer.reset();
//        while(true){
//            setCRServosIntakePower(0.5, 0.5);
//            if (timer.seconds() >= 20) {
//                setCRServosIntakePower(0, 0);
//                f = 2;
//                break;
//            }
//        }
//    }
//
//    public void getConeOut(){
//        timer = new ElapsedTime();
//        timer.reset();
//        while(true){
//            setCRServosIntakePower(-0.5, -0.5);
//            if (timer.seconds() >= 10) {
//                setCRServosIntakePower(0, 0);
//                f = 2;
//                break;
//            }
//        }
//    }
//
//    @Override
//    public void run() {
//        while(!Thread.currentThread().isInterrupted()) {
//            if (f == 0) {
//                getConeIn();
//            } else if (f == 1) {
//                getConeOut();
//            } else if (f == 2) {
//                ;
//                ;
//            }
//        }
//    }
}
