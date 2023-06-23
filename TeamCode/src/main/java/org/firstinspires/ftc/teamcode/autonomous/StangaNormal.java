package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.*;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.SignalPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import kotlin.jvm.functions.Function0;
import java.util.Random;

import java.util.Arrays;

@Autonomous(name = "Stanga test", group = "PP Rebuild")
public class StangaNormal extends LinearOpMode {
    public ElapsedTime timer = new ElapsedTime();
    public final SignalPipeline pipeline = new SignalPipeline(1.5F, new int[]{0, 1, 2});
    protected Camera camera;

    public int target = -1;

    public SampleMecanumDrive drive;

    public LifterEx lifter;
    public IntakeNou intake;

    public final Pose2d initialPoseLeft = new Pose2d(-36, -63, Math.toRadians(270));

    public TrajectorySequence toStaticAutoPos(@NonNull Pose2d _start){
        return drive.trajectorySequenceBuilder(_start)
            .UNSTABLE_addTemporalMarkerOffset(0d, () -> {
                // place preload here
                // actually e probabil mai bine ca dupa traiectorie sa aiba functie de placePreload()
            })
            .lineToLinearHeading(new Pose2d(-36, -10, Math.toRadians(200))).build();
    }

    /*public TrajectorySequence pivotCorrection(@NonNull Pose2d _start){
        return null;
    }*/ // asta era valabila daca aveam camera pe claw si detectam pole-ul

    public void place5Cones(){
        // for the moment this is just one cone (if it works at all)
        lifter.setTargetTicks(0, 500);
        intake.setTargetDeg(0, 20);

        intake.extendSlider(100, .4);
        intake.clawToggle(150);
        lifter.setTargetTicks(160, 1200);

        intake.extendSlider(180, 1d);
        intake.setTargetDeg(180, 200);
        intake.extendSlider(200, .4);

        sleep(200);
        //TrajectorySequence pivotCorrection = pivotCorrection(pose);
        //drive.followTrajectorySequence(pivotCorrection);
        //pose = pivotCorrection.end();
            
        // time delay was reset
        lifter.setTargetTicks(0, 500);
        intake.clawToggle(30);

        intake.extendSlider(100, .4);
        intake.setTargetDeg(200, 20);
    } 

    public TrajectorySequence toParkZone(@NonNull Pose2d _start, int _zone){
        int x = -36;

        if(_zone == -1){
            Random lozNorocos = new Random();
            _zone = lozNorocos.nextInt(3); // extragere la loto
            telemetry.addData("choosing random parking zone", _zone);
            telemetry.update();
        }

        if(_zone == 0) {
            // parking zone #1
            x = -10;
        } else if (_zone == 2){
            // parking zone #3
            x = -57;
        }
        // for parking zone 2, x stays the same

        return drive.trajectorySequenceBuilder(_start)
            .addTemporalMarker(0, () -> {
                intake.extendSlider(0, 1d);
                intake.clawOpen();
                intake.setTargetDeg(300, 0);
                lifter.setTargetTicks(500, 100);
                intake.rotateClawToAngle(0, 0);
            })
            .lineToLinearHeading(new Pose2d(x, -10, Math.toRadians(270)))
            .build();
    }
     

    public void grabPreload(){
        lifter.setTargetTicks(0, 100);
        intake.setTargetDeg(200, 30);
        intake.clawSetPosition(250, .4);
        intake.isClawOpen = false; // ensure calls to clawToggle actually toggle its state from closed to open
        //lifter.setTargetTicks(500, 500);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera(hardwareMap); // clasa noua wrapper ca sa nu mai copiem initDetection in toate autourile
        lifter = new LifterEx(hardwareMap);
        intake = new IntakeNou(hardwareMap, lifter);
        telemetry.setAutoClear(false); // nu mai sterge telemetria la Telemetry.update(); trb sa dai Telemetry.clear() ca sa stergi

        // init camera and start april tag detection
        camera.initDetection(pipeline, telemetry);
        sleep(1000);

        if (camera.ok){
            pipeline.startAprilTagDetection();
            telemetry.addLine("camera initialized sucessfully");
            telemetry.update();
        } else {
            telemetry.addLine("webcam failed! please RESTART!");
            telemetry.update();
        }

        // build trajectories (that can be pre-built)
        TrajectorySequence toStaticAutoPos = toStaticAutoPos(initialPoseLeft);

        while(! isStarted()){
            target = pipeline.targetFound;

            if (target != -1){
                telemetry.addLine("everythings alright. press START!");
                telemetry.update();
                pipeline.stopAprilTagDetection(); // end april tags detection
            }
        }

        intake.startIntakeThread();
        lifter.startLifterThread();
        intake.endClawProcessesForcibly();

        telemetry.clear();
        pipeline.stopAprilTagDetection();
        telemetry.addData("Sleeve", target + 1);
        TrajectorySequence toParkZone = toParkZone(toStaticAutoPos.end(), target);
        camera.close();

        grabPreload();
        sleep(3000);

        telemetry.addLine("going to static auto position");
        telemetry.update();

        drive.setPoseEstimate(initialPoseLeft);
        drive.followTrajectorySequence(toStaticAutoPos);
        // placePreload() maybe???
        
        /*
        sleep(1000);
        
        place5Cones();
        
        sleep(500);
       
        drive.followTrajectorySequence(toParkZone);
        */
        
        intake.endClawProcessesForcibly();
        intake.killIntakeThread();
        lifter.killLifterThread();
    }
}
