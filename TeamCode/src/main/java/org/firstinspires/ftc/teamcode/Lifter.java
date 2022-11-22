package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

public class Lifter implements Runnable {
    public DcMotorEx leftLifter;
    public DcMotorEx rightLifter;
    public Encoder lifterEncoder;
    public HardwareMap hardwareMap;
    private Telemetry telemetry;

    public PIDFController controllerDown = new PIDFController(new PIDCoefficients(1, 0, 0));
    public PIDFController controllerUp = new PIDFController(new PIDCoefficients(11, 0, 0));

    public volatile boolean kill = false;

    public volatile double targetTicks = 0;
    private double prevTicks = 0;

    private long lastMillis = 0;

    public Lifter(@NonNull HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        leftLifter = hardwareMap.get(DcMotorEx.class, "lifterLeft");
        rightLifter = hardwareMap.get(DcMotorEx.class, "lifterRight");

        lifterEncoder = new Encoder(leftLifter);

        leftLifter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLifter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLifterPower(double pow){
        leftLifter.setPower(pow);
        rightLifter.setPower(pow);
    }

    public double getCurrentPosition() {
        return leftLifter.getCurrentPosition();
    }

    public double getCorrectedVelocity() {
        return lifterEncoder.getCorrectedVelocity();
    }

    double wait = 0;

    public void setTargetTicks(double ticks) {
        this.targetTicks = ticks;
    }

    public volatile long waitLifter = 0;

    @Override
    public void run(){

        //ca sa astepte lifter-ul
        if(waitLifter > 0){
            try {
                Thread.sleep(waitLifter);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

        while (!kill && !Thread.currentThread().isInterrupted()) {

            //never run too fast
            if (SystemClock.uptimeMillis() - lastMillis < 100) {
                continue;
            }

            //set the last send time
            lastMillis = SystemClock.uptimeMillis();

            if (targetTicks != prevTicks) {
                //go to new target
                double target = targetTicks;
                double currentPosition = getCurrentPosition();
                double initialAbsError = Math.abs(currentPosition - target);

                if (initialAbsError < 50) { //inainte 100; posibil sa dea erori acest 25
                    prevTicks = targetTicks;
                    continue;
                }

                if (currentPosition < target) {
                    //go up
                    prevTicks = targetTicks;
                    controllerUp.reset();
                    controllerUp.setTargetPosition(target);

                    while (!Thread.currentThread().isInterrupted() && targetTicks == prevTicks) {

                        currentPosition = getCurrentPosition();

                        double correction = controllerUp.update(currentPosition) / (initialAbsError);
                        double power = Range.clip(correction, 0.1, 0.6);
                        setLifterPower(power);

                        telemetry.addData("target", target);
			            telemetry.addData("pos", currentPosition);
                        telemetry.addData("abserror", initialAbsError);
                        telemetry.addData(" going up - ticksCurrent", currentPosition);
                        telemetry.addData("correction", correction);
                        telemetry.addData("power", power);
                        telemetry.update();
                    }
                }
                else {
                    // go down
                    telemetry.log().clear();

                    prevTicks = targetTicks;
                    controllerDown.reset();
                    controllerDown.setTargetPosition(target);

                    while (!Thread.currentThread().isInterrupted() && targetTicks == prevTicks) {

                        currentPosition = getCurrentPosition();

                        double correction = controllerDown.update(currentPosition) / (initialAbsError);
                        double power = Range.clip(correction, -0.25, 0.0);
                        setLifterPower(power);

                        telemetry.addData("ticksCurrent", currentPosition);
			            telemetry.addData("pos", currentPosition);
                        telemetry.addData("target", target);
                        telemetry.addData("correction", correction);
                        telemetry.addData("power", power);
                        telemetry.update();
                    }
                }
            }
            else {
                prevTicks = targetTicks;
            }
        }
    }
}
