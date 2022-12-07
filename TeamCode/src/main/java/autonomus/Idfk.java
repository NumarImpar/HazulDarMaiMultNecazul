package autonomus;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name ="autoPark", group = "_0auto")
public class Idfk extends LinearOpMode {
//clasa automonie parcare pe pozitia 2

    public SampleMecanumDrive drive;

    public ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        timer = new ElapsedTime();

            waitForStart();
            timer.reset();

            while(opModeIsActive()){
                drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
                while(timer.milliseconds() < 1750){
                    //wait for time to pass
                }
                drive.setMotorPowers(0,0,0,0);
                break;
            }

        }

    }
