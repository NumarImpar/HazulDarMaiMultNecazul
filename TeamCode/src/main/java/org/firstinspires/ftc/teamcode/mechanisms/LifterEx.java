package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;
import static java.lang.Math.abs;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.roadrunner.control.*;
import com.acmerobotics.roadrunner.profile.*;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import kotlin.jvm.functions.Function2;

public class LifterEx implements Runnable {

    protected DcMotorEx leftLifter;
    protected DcMotorEx rightLifter;
	public Thread lifterThread;

    public double getCurrent(CurrentUnit amps) {
        return getCurrent(CurrentUnit.AMPS);
    }

    public static enum LIFTER_LEVEL {
        HIGH(2700), LOW(1100), MID(1800), DOWN(0);

        public int ticks;

        LIFTER_LEVEL(int ticks) {
            this.ticks = ticks;
        }
    }

    public LifterEx(@NonNull HardwareMap _hardwareMap) {

        leftLifter = _hardwareMap.get(DcMotorEx.class, "lifterLeft");
        rightLifter = _hardwareMap.get(DcMotorEx.class, "lifterRight");

		// set direction
        leftLifter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLifter.setDirection(DcMotorSimple.Direction.REVERSE);

		// set brake
        leftLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// reset encoders
        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		// run without encoder
        leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		lifterThread = new Thread(this);
        kill = false;
    }

    public void setPow(double _pow) {
        leftLifter.setPower(_pow);
        rightLifter.setPower(_pow);
    }

    public int getCurrentPosition() {
        return leftLifter.getCurrentPosition();
    }

	public double getCurrentVelocity(){
		return leftLifter.getVelocity();
	}

	public void startLifterThread(){
		lifterThread.start();
	}

    public void killLifterThread() {
		lifterThread.interrupt();
        kill = true;
		setPow(0);
    }

	public boolean isIdle(){
		if(manual){
			return manualPower == 0;
		} else if (! kill) {
			return targetTicks == lastTargetTicks;
		} else {
			return true;
		}
	}

    public void setTargetTicks(long _wait, int _target) {
        if(_wait > 0){
			new Thread(() -> {
				try{
					Thread.sleep(_wait);
				} catch (InterruptedException e){
					e.printStackTrace();
					return;
				}

				targetTicks = _target;
                stay = false;
			}).start();
		} else {
				targetTicks = _target;
                stay = false;
		}
    }

	public void toggleManual(){
		if(manual){
			targetTicks = getCurrentPosition();
			manual = false;
			manualPower = 0;
		} else {
			manual = true;
			/*if (manualPower == 0 && getCurrentPosition() > 500){
				manualPower = kG;
			}*/ // might not be needed at all
		}
	}

    public void setManualPower(double _power){
		manualPower = _power;
    }

	public int targetTicks = 0, lastTargetTicks = 0;

	public boolean kill = false;
	public boolean manual = false;
	public double manualPower = 0;
    public double correction = 0;

    /*
	public final double TICKS_TO_CM = 0.0374177;
	public final double K = 27.5; // lungimea barii de jos de la v4bar
	public final double X = 13.5; // lungimea legaturii de la mijloc de la v4bar
    */
                                  
    public boolean stay = true;

    public final double kG = 0.32, kG2 = 0.05;
    public final double kP1 = 0.0002, kD1 = 0.00008, kI1 = 0;
    public final double kP2 = 0.0002, kD2 = 0, kI2 = 0;
    public final PIDCoefficients pid1 = new PIDCoefficients(kP1, kI1, kD1);
    public final PIDCoefficients pid2 = new PIDCoefficients(kP2, kI2, kD2);
    public final PIDFController pidf2 = new PIDFController(pid2, 0, 0, 0, (x, v) -> 0d);

	public boolean goesUp(){
		return lastTargetTicks < targetTicks;
	}

    public volatile boolean isMotorIntakeMoving = false;
    public final double kIntake = -16E-3;
    public volatile double kIntakeFactor = 1d;

	public Function2<Double, Double, Double> kFun = (x, v) -> {
		//double H = getCurrentPosition() * TICKS_TO_CM;
        //double sinTheta = (H - X)/(2 * K);
        //double cosThetaSquared = 1 - sinTheta * sinTheta;
		return ((goesUp()) || ( isIdle() && getCurrentPosition() > 150) )?(kG):(kG2) /*- ((goesUp())?(1d):(-1d) * kE * Range.clip((H / 2) / (sqrt(H * H / 4 + K * K * cosThetaSquared), -1d, 1d)))*/;
	};


    public final PIDFController pidf1 = new PIDFController(pid1, 0, 0, 0, kFun);

    @Override
    public void run() {
		while(! kill){
			if(manual){
				setPow(manualPower);
				try{	
					Thread.sleep(50);
				} catch (InterruptedException e){
					e.printStackTrace();
				}
			} else {

                //setPow(kFun.invoke(0d, 0d));

				if(targetTicks != lastTargetTicks){
					pidf1.reset(); // reset integral
					int currentTicks = getCurrentPosition();
                    pidf1.setTargetPosition(targetTicks);

                    boolean exit = false;

					while((! kill) && (! exit) && (! manual)){
                        if(targetTicks < lastTargetTicks){
                            exit = currentTicks - 50 <= targetTicks;
                        } else {
                            exit = currentTicks + 50 >= targetTicks;
                        }
						correction = pidf1.update(targetTicks - currentTicks);
                        double down = (targetTicks < lastTargetTicks)?(-0.8d):(1d);
						setPow(Range.clip(correction * down, -0.2, 0.8));
						currentTicks = getCurrentPosition();
					}

                    stay = true;
					lastTargetTicks = targetTicks;
				} else if(stay){
                    pidf2.reset();
                    pidf2.setTargetPosition(targetTicks);
                    int currentTicks = getCurrentPosition();

                    while(stay && (! kill) && (! manual)){
                        correction = pidf2.update(targetTicks - currentTicks);
                        if(isMotorIntakeMoving){
                            correction += kIntake * kIntakeFactor * Math.pow((1 + Range.clip(currentTicks / 500, 0, 4)), 2);
                        }
                        if(currentTicks < 0){
                            correction = Math.abs(correction);
                        }

                        if (currentTicks < targetTicks){
                            correction *= Range.clip(Range.clip(Math.abs(currentTicks - targetTicks) / 200, 0, 1) * 3, 1, 3);
                        }

                        if(Math.abs(currentTicks - targetTicks) > 200){
                            correction *= 3;
                        }

                        setPow(Range.clip(correction, -0.2, 0.15));
                        
                        currentTicks = getCurrentPosition();
                    }
                }
			}
		}

		// when killed
		setPow(0);
    }

}

