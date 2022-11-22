import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.*;
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
        double lastPos = 0.5;
        int poz = 0;

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
            if(controller2.YOnce() && !gamepad2.start){
                lifter.setTargetTicks(2700);
            }

            //cerc - poz intermediare
            if(controller2.BOnce() && !gamepad2.start){
                lifter.setTargetTicks(1900);
            }

            //patrat - poz intermediara
            if(controller2.XOnce() && !gamepad2.start){
                lifter.setTargetTicks(800);
            }

            //arrow up arrow down brat fata spate --

//            if (controller2.dpadUpOnce()){
//                intake.moveIntake(0);
//                //intake.switchState(true);
//                alrup = false;
//            }
//            if (controller2.dpadDownOnce()){
//                if (alrup){
//                    intake.moveIntake(0.7);
//                    alrup = false;
//                } else {
//                    intake.moveIntake(0.5);
//                    alrup = true;
//                }
//            }
            

            if(controller2.dpadDownOnce()){
//                for(double i=lastPos; i>-1; i = i-0.005){
                    intake.moveIntake(-1);
                 //   lastPos = i;
               // }
            }
            if(controller2.dpadUpOnce()){
               // for(double i=lastPos; i<1; i = i+0.005){
                    intake.moveIntake(1);
                //    lastPos = i;
               // }
            }


//            //patrat - duce lift automat sus max
//            if (controller2.XOnce() && flag == 0 && !gamepad2.start) {
//                lifter.setTargetTicks(2500);
//                flag = 1;
//            }


            //urca lifter-ul doar manual !!!!

            //left bumper - auto target pos lifter = 50;
            if(controller2.leftBumper()){
                lifter.setTargetTicks(50);
            }

            //patrat - duce lift ul jos si baga inauntru de la orice pozitie
            if (controller2.XOnce() && flag == 1 && !gamepad2.start) {
               lifter.setTargetTicks(1500);
                flag = 0;
                intake.moveIntakeArm(1000, 0);
                lifter.waitLifter = 500;
                lifter.waitLifter = 0;
                lifter.setTargetTicks(50);
            }
            //POSIBIL SA NU MEARGA FIX FAZA CU WAIT. DACA NU MERGE,
            // DOAR LASA LIFTERUL SA COBOARE CU AIA INAUNTRU DOAR DE LA MAXIM

            // x rotite intake
            //get cone in
           if (controller2.AOnce()&& !gamepad2.start) {
                intake.spinForwardForMs(800);
                flag2= 1;

            }

            //get cone out
            if (controller2.dpadRightOnce()&& !gamepad2.start){
                intake.spinReverseForMs(800);
                flag2 = 0;

            }

//            //bumper dreapta / stanga - ridica lasa lifterul - manual
//            if(controller2.rightBumper()){
//                if(lifter.getCurrentPosition() < 2400) {
//                    lifter.setTargetTicks(lifter.getCurrentPosition() + 60);
//                }
//            }
//
//            if(controller2.leftBumper()){
//                if(lifter.getCurrentPosition() > 70) {
//                    lifter.setTargetTicks(lifter.getCurrentPosition() - 5);
//                }
//            }

            if (controller1.rightBumper()) {
                handleDrivingSlowed();
            } else handleDriving();
        }

        lifterThread.interrupt();

    }

    private void handleDriving() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * 0.7,
                        -gamepad1.left_stick_x * 0.7,
                        gamepad1.right_stick_x * 0.7
                        //forward version has - at right_stick
                )
        );
    }

    private void handleDrivingSlowed(){
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * 0.35,
                        -gamepad1.left_stick_x * 0.35,
                        gamepad1.right_stick_x * 0.35
                )
        );
    }

}
