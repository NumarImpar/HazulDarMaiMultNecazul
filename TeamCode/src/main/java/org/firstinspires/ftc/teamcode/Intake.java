package org.firstinspires.ftc.teamcode;

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

    public ServoImplEx leftServoIntake;
    public ServoImplEx rightServoIntake;

    public Intake(HardwareMap _hardwareMap){
        leftCRServoIntake = _hardwareMap.get(CRServo.class, "leftCRServoIntake");
        rightCRServoIntake = _hardwareMap.get(CRServo.class, "rightCRServoIntake");

	    leftServoIntake = _hardwareMap.get(ServoImplEx.class, "leftServoSwing");
        rightServoIntake = _hardwareMap.get(ServoImplEx.class, "rightServoSwing");

	    leftServoIntake.setPwmRange(new PwmRange(500, 2500));
	    rightServoIntake.setPwmRange(new PwmRange(500, 2500));

        leftServoIntake.setPosition(0);
        rightServoIntake.setPosition(1);

        leftCRServoIntake.setDirection(CRServo.Direction.REVERSE);
        rightCRServoIntake.setDirection(CRServo.Direction.FORWARD);
    }

    public void setCRServosIntakePower(double _power1, double _power2){
        leftCRServoIntake.setPower(_power1);
        rightCRServoIntake.setPower(_power2);
    }


    public void startSpin() {
        setCRServosIntakePower(0.3, 0.3);
    }

    public void stopSpin() {
        setCRServosIntakePower(0.0, 0.0);
    }

    public void startReverseSpin(){
        setCRServosIntakePower(-0.3, -0.3);
    }

    public Thread spinForwardForMs(long sleep, long ms) {

        if(ms <= 0) return new Thread();

        Thread t = new Thread(() -> {

            try {
                Thread.sleep(sleep);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

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

    public Thread spinReverseForMs(long sleep, long ms) {

        if(ms <= 0) return new Thread();

        Thread t = new Thread(() -> {

            try {
                Thread.sleep(sleep);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

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

    public double _pos = 0;
    public void goToPos(double pos){_pos = pos;}
    public boolean threadRunning = false;
    public volatile boolean kill = false;

    public Thread moveIntakeArm(long wait, double pos) {
	goToPos(pos);
	if (threadRunning) {return new Thread();}
        Thread t  = new Thread(() -> {
            if (wait != 0) {
                try {
                    Thread.sleep(wait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

	    while (!kill){
	    while (_pos == 0d && !kill){
		if (Drive.virginIntake){
                leftServoIntake.setPosition(0.3);
                rightServoIntake.setPosition(0.7);
		} else { 
                leftServoIntake.setPosition(0);
                rightServoIntake.setPosition(1);
		}
	    }

	    while (_pos != 0 && !kill){
                leftServoIntake.setPosition(_pos);
                rightServoIntake.setPosition(1 - _pos);
	    }}
        });
	t.start();
	threadRunning = true;
        return t;
    }


    public void moveIntake(double pos){
        leftServoIntake.setPosition(Range.clip(pos, 0, 1));
	    rightServoIntake.setPosition(Range.clip(1-pos, 0, 1));
    }
}
