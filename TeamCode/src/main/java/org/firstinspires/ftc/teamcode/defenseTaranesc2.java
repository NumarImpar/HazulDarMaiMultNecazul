package autonomus;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


//------JUST PARK USING TIMER + DETECTION----------------

@Autonomous(name ="defense2", group = "_1auto")
public class defenseTaranesc2 extends LinearOpMode {

    @Override
    public void runOpMode(){
         SampleMecanumDrive drive=new SampleMecanumDrive(hardwareMap);


        waitForStart();



                drive.setMotorPowers(0.7, 0.7, 0.7, 0.7);
                sleep(2600);
                drive.setMotorPowers(0, 0, 0, 0);
                sleep(1000);
                drive.setMotorPowers(-0.4, -0.4, -0.4, -0.4);
                sleep(2500);
                drive.setMotorPowers(0, 0, 0, 0);

                sleep(1000);




    }
}
