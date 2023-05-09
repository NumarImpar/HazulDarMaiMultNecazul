package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.lib.ControllerInput;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;

@Config
@TeleOp
public class IntakePIDTuner extends LinearOpMode {
    public ControllerInput controller;
    Intake intake;
    protected Thread intakeThread;

    public static double kP = 10.0;
    public static double kI = 0.05;
    public static double kD = 0.0;
    public static double f = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new ControllerInput(gamepad1);

        PIDFCoefficients coefficients = intake.getPIDFCoefficients();
        telemetry.addLine("Current PIDF values");
        telemetry.addData("kP", coefficients.p);
        telemetry.addData("kI", coefficients.i);
        telemetry.addData("kD", coefficients.d);
        telemetry.addData("f", coefficients.f);
        telemetry.update();

        waitForStart();
        intakeThread = new Thread(intake);
        intakeThread.start();

        while (opModeIsActive()) {
            controller.update();

            telemetry.addData("Position", intake.getCurrentPosition());
            telemetry.update();

            if (controller.rightBumperOnce()) {
                intake.pidfCoefficients = new PIDFCoefficients(kP, kI, kD, f); // la fiecare apasare se actualizeaza coeficientii
                intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.INSIDE.ticks);
            }

            if (controller.leftBumperOnce()) {
                intake.pidfCoefficients = new PIDFCoefficients(kP, kI, kD, f); // la fiecare apasare se actualizeaza coeficientii
                intake.swingSetTargetTicks(0, Intake.INTAKE_STATE.OUTSIDE.ticks);
            }
        }
    }
}
