package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {

    static public int pozitieParcare = 1;  //1 2 sau 3

    // Red - Blue Terminal initial pose
    static Pose2d initialPoseRedBlueTerminal = new Pose2d(36, -63, Math.toRadians(90));

    // Red Red Terminal initial pose
    static Pose2d initialPoseRedRedTerminal = new Pose2d(-36, -63, Math.toRadians(90));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 6, Math.toRadians(180), Math.toRadians(180), 10.2362)
                .setDimensions(14.56, 15.55).build();

        //myFirstBot.followTrajectorySequence(REDAllianceBlueTerminal(myFirstBot, initialPoseRedBlueTerminal, pozitieParcare ));
        myFirstBot.followTrajectorySequence(REDAllianceRedTerminal(myFirstBot, initialPoseRedRedTerminal, pozitieParcare));

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFirstBot)
                .start();
    }

    public static TrajectorySequence REDAllianceBlueTerminal(RoadRunnerBotEntity bot, Pose2d initialPose, int pozitieParcare) {
        DriveShim drive = bot.getDrive();

        if (pozitieParcare == 1) {
            return drive.trajectorySequenceBuilder(initialPose)

                    //-----GO HIGH AND LEAVE PRELOAD--------
                    //.lineTo(new Vector2d(34, -40))
                    //.splineToLinearHeading(new Pose2d(34, -18, Math.toRadians(210)), Math.toRadians(90)) //-23y, 90 2math.torad

                    .lineToSplineHeading(new Pose2d(34.5, -6, Math.toRadians(145)))

                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(2500);
                        //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                        //intake.setReverseSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 1500);
                    })

                    //---------GO COLLECT CONES (1) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //intake.goBack(wait, pos)
                        //intake.setReversedSpin(wait, 800);
                    })

                    //---------GO COLLECT CONES (2) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //intake.goBack(wait, pos)
                        //intake.setReversedSpin(wait, 800);
                    })

                    //------PARKING FROM HIGH to pos 1--------
                    .setReversed(true)
                    //.splineToLinearHeading(new Pose2d(13, -15, Math.toRadians(0)), Math.toRadians(-150))

                    //sa vedem care metda de parcare e cea mai eficienta in combinatie cu intake ul ala
                    .splineToLinearHeading(new Pose2d(12, -15, Math.toRadians(85)), Math.toRadians(0))

                    .build();

        } else if (pozitieParcare == 2) {

            return drive.trajectorySequenceBuilder(initialPose)

                    //-----GO MID AND LEAVE PRELOAD--------

                    //.lineTo(new Vector2d(34, -40))
                    //.splineToLinearHeading(new Pose2d(34, -18, Math.toRadians(210)), Math.toRadians(90)) //-23y, 90 2math.torad

                    //de fapt, aici o sa incerc sa pun si preload-ul pe high
                    //sper sa nu omoare odometria daca atinge ground-ul, e de testat
                    .lineToSplineHeading(new Pose2d(34, -7, Math.toRadians(145)))

                    //.lineToLinearHeading(new Pose2d(34, -18, Math.toRadians(210)))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(2500);
                        //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                        //intake.setReverseSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 1500);
                    })

                    //---------GO COLLECT CONES (1) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //intake.goBack(wait, pos)
                        //intake.setReversedSpin(wait, 800);
                    })

                    //---------GO COLLECT CONES (2) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //intake.goBack(wait, pos)
                        //intake.setReversedSpin(wait, 800);
                    })

                    //------PARKING FROM HIGH to pos 2--------

                    .setReversed(true)
                    .lineToLinearHeading(new Pose2d(36, -19, Math.toRadians(90)))
                    .build();
        }

//default pos 3
        return drive.trajectorySequenceBuilder(initialPose)

                //-----GO MID AND LEAVE PRELOAD--------
                //.lineTo(new Vector2d(34, -40))
                //.splineToLinearHeading(new Pose2d(34, -18, Math.toRadians(210)), Math.toRadians(90)) //-23y, 90 2math.torad

                //----GO HIGH AND LEAVE PRELOAD---- dar e de testat
                .lineToSplineHeading(new Pose2d(34, -7, Math.toRadians(145)))

                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(2500);
                    // intake.goBack(wait, pos) sa se duca in spate la o pozitie
                    //intake.setReverseSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 1500);
                })

                //---------GO COLLECT CONES (1) ------
                .setReversed(true)
                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(50);
                    //intake.goFront(wait, pos) //make intake go in the front part of the robot
                    //intake.setSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 2500);
                })

                //------GO HIGH FROM STACK -----------
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //intake.goBack(wait, pos)
                    //intake.setReversedSpin(wait, 800);
                })

                //---------GO COLLECT CONES (2) ------
                .setReversed(true)
                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(50);
                    //intake.goFront(wait, pos) //make intake go in the front part of the robot
                    //intake.setSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 2500);
                })

                //------GO HIGH FROM STACK -----------
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(34, -6, Math.toRadians(150)), Math.toRadians(130))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //intake.goBack(wait, pos)
                    //intake.setReversedSpin(wait, 800);
                })

                //------PARKING FROM HIGH to pos 3 default--------
                //.lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(58, -13), Math.toRadians(0))

                .build();
    }

    public static TrajectorySequence REDAllianceRedTerminal(RoadRunnerBotEntity bot, Pose2d initialPoseRedRed, int pozitieParcare){
        DriveShim drive = bot.getDrive();

        if(pozitieParcare == 1){
            return drive.trajectorySequenceBuilder(initialPoseRedRedTerminal)

                    //-----GO MID AND LEAVE PRELOAD--------
                    //.lineTo(new Vector2d(-34, -40))
                    //.splineToLinearHeading(new Pose2d(-34, -18, Math.toRadians(-30)), Math.toRadians(90)) //-23y, 90 2math.torad

                    //------GO HIGH AND LEAVE PRELOAD-----
                    .lineToSplineHeading(new Pose2d(-34, -7, Math.toRadians(45)))

                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(2500);
                        //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                        //intake.setReverseSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 1500);
                    })

                    //---------GO COLLECT CONES (1) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    //.splineTo(new Vector2d(-32, -18), Math.toRadians(-30)) //40, 50
                    .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

                    //---------GO COLLECT CONES (2) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
                    //.UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

//
//                    //------PARKING FROM HIGH to pos 1--------
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-60, -13, Math.toRadians(0)), Math.toRadians(-170))


                    .build();

        } else if(pozitieParcare == 2){
            return drive.trajectorySequenceBuilder(initialPoseRedRedTerminal)

                    //-----GO MID AND LEAVE PRELOAD--------
                    //.lineTo(new Vector2d(-34, -40))
                    //.splineToLinearHeading(new Pose2d(-34, -18, Math.toRadians(-30)), Math.toRadians(90)) //-23y, 90

                    //------GO HIGH AND LEAVE PRELOAD-----
                    .lineToSplineHeading(new Pose2d(-34, -7, Math.toRadians(45)))

                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(2500);
                        //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                        //intake.setReverseSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 1500);
                    })

                    //---------GO COLLECT CONES (1) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

                    //---------GO COLLECT CONES (2) ------
                    .setReversed(true)
                    .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                        //lifter.setTargetPosition(50);
                        //intake.goFront(wait, pos) //make intake go in the front part of the robot
                        //intake.setSpin(wait, 800);
                        //lifter.setTargetPosition(wait, 2500);
                    })

                    //------GO HIGH FROM STACK -----------
                    .setReversed(false)
                    .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(45)), Math.toRadians(50))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

//
//                    //------PARKING FROM HIGH to pos 2--------
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-35, -13, Math.toRadians(0)), Math.toRadians(-170))


                    .build();

        }
        return drive.trajectorySequenceBuilder(initialPoseRedRedTerminal)

                //-----GO MID AND LEAVE PRELOAD--------
                //.lineTo(new Vector2d(-34, -40))
                //.splineToLinearHeading(new Pose2d(-34, -18, Math.toRadians(-30)), Math.toRadians(90)) //-23y, 90 2math.torad

                //------GO HIGH AND LEAVE PRELOAD-----
                .lineToSplineHeading(new Pose2d(-34, -7, Math.toRadians(45)))

                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(2500);
                    //intake.goBack(wait, pos) sa se duca in spate la o pozitie
                    //intake.setReverseSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 1500);
                })

                //---------GO COLLECT CONES (1) ------
                .setReversed(true)
                .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(50);
                    //intake.goFront(wait, pos) //make intake go in the front part of the robot
                    //intake.setSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 2500);
                })

                //------GO HIGH FROM STACK -----------
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })

                //---------GO COLLECT CONES (2) ------
                .setReversed(true)
                .splineTo(new Vector2d(-61, -12), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
                    //lifter.setTargetPosition(50);
                    //intake.goFront(wait, pos) //make intake go in the front part of the robot
                    //intake.setSpin(wait, 800);
                    //lifter.setTargetPosition(wait, 2500);
                })

                //------GO HIGH FROM STACK + parking 3-----------
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-31, -6, Math.toRadians(40)), Math.toRadians(45))
//                    .UNSTABLE_addTemporalMarkerOffset(0.9, () -> {
//                        //intake.goBack(wait, pos)
//                        //intake.setReversedSpin(wait, 800);
//                    })


                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, -16, Math.toRadians(90)), Math.toRadians(-40))
                .build();
    }
}