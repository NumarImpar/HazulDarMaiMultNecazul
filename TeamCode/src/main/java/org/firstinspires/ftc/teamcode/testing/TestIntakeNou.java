package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeNou;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;
import org.firstinspires.ftc.teamcode.mechanisms.LifterEx;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "intake nou testing")
public class TestIntakeNou extends LinearOpMode {
    public DcMotorEx intakeMtr;
    public IntakeNou intake;
    public ControllerInput controller2;
    public LifterEx lifter;
    public Thread lifterThread;

    @Override
    public void runOpMode(){
        lifter = new LifterEx(hardwareMap);
        intake = new IntakeNou(hardwareMap, lifter);
        controller2 = new ControllerInput(gamepad2);

        intake.endClawProcessesForcibly();

        waitForStart();

        intake.startIntakeThread();
        lifter.startLifterThread();

        int pos = intake.getCurrentPosition();
        int deg = intake.degFromTicks(pos);

        while(opModeIsActive()){
            pos = intake.getCurrentPosition();
            controller2.update();

            //lifter.setTargetTicks(0, Lifter.LIFTER_LEVEL.MID.ticks);

            if(controller2.crossOnce()){
                lifter.setTargetTicks(0, 1000);
            }

            if(controller2.triangleOnce()){
                lifter.setTargetTicks(0, 1800);
            }

            if(controller2.rightBumper()){
                lifter.setTargetTicks(0, 2700);
            }

            if(controller2.circleOnce()){
                intake.setTargetDeg(0, 0);
            }

            if(controller2.dpadLeftOnce()){
                intake.setTargetDeg(0, 200);
            }

            if(controller2.dpadDownOnce()){
                intake.extendSlider(0, 0.4);

            }

            if(controller2.dpadUpOnce()){
                intake.extendSlider(0, 1);

            }

            if(controller2.squareOnce()){
                lifter.setTargetTicks(0, 500);
            }

            telemetry.addData("targetTicks - intake: ", intake.targetTicks);
            telemetry.addData("last targetTicks - intake: ", intake.lastTargetTicks);
            telemetry.addData("target - lifter", lifter.targetTicks);
            telemetry.addData("last target - lifter", lifter.lastTargetTicks);
            telemetry.addData("currentTicks - lifter: ", lifter.getCurrentPosition());
            telemetry.addData("kFun:", lifter.kFun.invoke(0d, 0d));
            telemetry.addData("correction:", lifter.correction);
            telemetry.addData("current deg", intake.getCurrentDeg());
            telemetry.addData("stay power", intake.stayPower);
            telemetry.addData("intake moving", lifter.isMotorIntakeMoving);

            if(lifter.getCurrentPosition() > 3500){
                lifter.killLifterThread();
                telemetry.addLine("killed");
            } else if (lifter.getCurrentPosition() < -150){
                lifter.killLifterThread();
                telemetry.addLine("killed");
            }

            //telemetry.addData("ok ", intake.ok);

            telemetry.update();
        }

        intake.endClawProcessesForcibly();
        intake.killIntakeThread();
        lifter.killLifterThread();
    }
}
