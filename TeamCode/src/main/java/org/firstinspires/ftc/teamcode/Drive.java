import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.ControllerInput;

@TeleOp
public class Drive extends LinearOpMode {
    Intake intake;
    Lifter lifter;
    SampleMecanumDrive drive;
    DcMotorEx LF, LB, RF, RB;
    ControllerInput controller1, controller2;

    int flag = 0;
    int flag2 = 0;

    public static int forward = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        lifter = new Lifter(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        controller1 = new ControllerInput(gamepad1);
        controller2 = new ControllerInput(gamepad2);

        LF = drive.LF;
        LB = drive.LB;
        RF = drive.RF;
        RB = drive.RB;

        waitForStart();

        Thread lifterThread = new Thread(lifter);
        lifterThread.start();
        boolean alrup = false;
        while (opModeIsActive()) {

            controller1.update();
            controller2.update();

            //triunghi - poz intermediare
            if(controller2.BOnce() && !gamepad2.start){
                lifter.setTargetTicks(1700);
            }

            //cerc - poz intermediare
            if(controller2.YOnce() && !gamepad2.start){
                lifter.setTargetTicks(500);
            }

            //arrow up arrow down brat fata spate --
            if (controller2.dpadUpOnce()){
                intake.moveIntake(0);
                alrup = false;
            }
            if (controller2.dpadDownOnce()){
                if (alrup){
                    intake.moveIntake(0.7);
                    alrup = false;
                } else {
                    intake.moveIntake(0.5);
                    alrup = true;
                }
            }

            //patrat - duce lift automat sus/jos max
            if (controller2.XOnce() && flag == 0 && !gamepad2.start) {
                lifter.setTargetTicks(2500);
                flag = 1;
            }

            if (controller2.XOnce() && flag == 1 && !gamepad2.start) {
                lifter.setTargetTicks(50);
                flag = 0;
            }

            // x rotite intake
            //get cone in
           if (controller2.AOnce() && flag2 == 0 && !gamepad2.start) {
                intake.spinForwardForMs(300);
                flag2= 1;

            }

            //get cone out
            if (controller2.AOnce() && flag2 == 1 && !gamepad2.start){
                intake.spinReverseForMs(300);
                flag2 = 0;

            }

            //bumper dreapta / stanga - ridica lasa manual
            if(controller2.rightBumper()){
                if(lifter.getCurrentPosition() < 2400) {
                    lifter.setTargetTicks(lifter.getCurrentPosition() + 30);
                }
            }

            if(controller2.leftBumper()){
                if(lifter.getCurrentPosition() > 30) {
                    lifter.setTargetTicks(lifter.getCurrentPosition() - 30);
                }
            }


            handleDriving();
            if (controller1.leftBumper()) {
                handleDrivingSlowed();
            }
        }

        lifterThread.interrupt();

    }

    private void handleDriving() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * 0.6,
                        -gamepad1.left_stick_x * 0.6,
                        -gamepad1.right_stick_x * 0.6
                )
        );
    }

    private void handleDrivingSlowed(){
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * 0.3,
                        -gamepad1.left_stick_x * 0.3,
                        -gamepad1.right_stick_x * 0.3
                )
        );
    }
}
