package com.example.meepmeepdeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepDeep {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-34, -60, Math.toRadians(90)))
                        // starting position: robot's right side aligned with seam between second and third tile
                        .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))
                        // robot moves in front of net zone, put sample already loaded into basket
                        // Crates a path to set coordinates and sets heading to 45deg,
                        .waitSeconds(3) //waits 3 seconds: this time will be for the tower to drop in the sample

                        .lineToLinearHeading(new Pose2d(-49, -41, Math.toRadians(90)))
                        .waitSeconds(1)
                        // pick up first spike (yellow sample)
                        .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))
                        /* TO DO:
                        - get tower working in autonomous
                        - get arm working in autonomous
                        - get servos working */
                        .waitSeconds(3)

                        .lineToLinearHeading(new Pose2d(-59, -41, Math.toRadians(90)))

                        .waitSeconds(1)

                        .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))

                        .waitSeconds(3)
                        // in front of yellow sample next to the wall
                        .lineToLinearHeading(new Pose2d(-58, -41, Math.toRadians(100)))

                        .waitSeconds(1)

                        .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))

                        .waitSeconds(3)

                        .lineToLinearHeading(new Pose2d(-26, 0, Math.toRadians(0)))

                        .waitSeconds(3)

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}