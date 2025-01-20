package org.firstinspires.ftc.teamcode.drive.opmode;



// RR-specific imports

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "PathTest")
public class PathTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system (uses Road Runner's SampleMecanumDrive)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Define the starting pose of the robot
        Pose2d startPose = new Pose2d(-34, -60, Math.toRadians(90));

        // Set the robot's starting position in the drive system
        drive.setPoseEstimate(startPose);

        // Create the trajectory sequence
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                // Move to high junction and simulate scoring
                // starting position: robot's right side aligned with seam between second and third tile
                .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))
                // robot moves in front of net zone, put sample already loaded into basket
                // Crates a path to set coordinates and sets heading to 45deg,
                .waitSeconds(3) //waits 3 seconds: this time will be for the tower to drop in the sample

                .lineToLinearHeading(new Pose2d(-48, -41, Math.toRadians(90)))
                .waitSeconds(1)
                // pick up spike 1
                .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45)))
                /* TO DO:
                - get tower working in autonomous
                - get arm working in autonomous
                - get servos working */
                .waitSeconds(3)
                // spike 2
                .lineToLinearHeading(new Pose2d(-58, -41, Math.toRadians(90)))
                //
                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45))) // drop off

                .waitSeconds(3)
                // spike 3: in front of yellow sample next to the wall
                .lineToLinearHeading(new Pose2d(-59, -41, Math.toRadians(100)))

                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(-54, -53, Math.toRadians(45))) // drop off

                .waitSeconds(3)

                .lineToLinearHeading(new Pose2d(-26, 0, Math.toRadians(0))) // park by submersible

                .waitSeconds(3)

                .build();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            // Follow the trajectory sequence
            drive.followTrajectorySequence(trajectorySequence);
        }
    }
}
