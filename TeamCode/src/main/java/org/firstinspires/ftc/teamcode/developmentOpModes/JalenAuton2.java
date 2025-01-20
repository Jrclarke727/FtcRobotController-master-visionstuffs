package org.firstinspires.ftc.teamcode.developmentOpModes;

//openCV Imports

import org.firstinspires.ftc.teamcode.vision.RotatedBoxesBlue;
import org.firstinspires.ftc.teamcode.vision.RotatedBoxesRed;
import org.firstinspires.ftc.teamcode.vision.RotatedBoxesYellow;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//ftcdashboard
import com.acmerobotics.dashboard.FtcDashboard;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;


@Config
@Autonomous(name = "JalenAuton2")
public class JalenAuton2 extends LinearOpMode {
    //declare Pipeline used
    public static String selectedPipeline = "Blue"; // "Blue", "Red", or "Yellow"
    //declare adjustment power
    public static final double ADJUSTMENT_POWER = 0.2;
    //declare cX adjustment tolerance
    public static final double X_TOLERANCE = 10;
    private static final int TOWER_MAX_POSITION = 1900;
    private static final int TOWER_MIN_POSITION = 0;

    private static final int ARM_MIN_POSITION = 40;
    private static final int ARM_MAX_POSITION = 320;

    private static final double BUCKET_TAKE_POSITION = 0;
    private static final double BUCKET_DROP_POSITION = 1;

    private static final double CLAW_CLOSED_POSITION = 0.3;
    private static final double CLAW_OPEN_POSITION = 0.8;

    private static final double WRIST_INTAKE_POSITION = 0.7;
    private static final double WRIST_DROP_POSITION = 0.1;
    private DcMotor tower = null;
    private DcMotor arm = null;
    private Servo bucket = null;
    private Servo claw = null;
    private Servo wrist = null;
    public int targetTower = 0;
    public int targetArm = 0;

    ////////////////////////////////////////AUTONOMOUS///////////////////////////////////////////////
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system (uses Road Runner's SampleMecanumDrive)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // tower init
        tower = hardwareMap.get(DcMotor.class, "tower");
        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setDirection(DcMotor.Direction.REVERSE);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tower.setTargetPosition(targetTower);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setPower(1);
        // arm init
        arm = hardwareMap.get(DcMotor.class, "arm");
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setTargetPosition(targetArm);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        // servos init
        bucket = hardwareMap.get(Servo.class, "bucket");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_OPEN_POSITION);



        // Initialize vision pipeline
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "controlHubCam")
        );

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("Camera opened successfully");
                telemetry.update();
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error opening camera: " + errorCode);
                telemetry.update();
            }
        });

        // Select pipeline dynamically
        RotatedBoxesBlue pipelineBlue = null;
        RotatedBoxesRed pipelineRed = null;
        RotatedBoxesYellow pipelineYellow = null;

        if ("Blue".equalsIgnoreCase(selectedPipeline)) {
            pipelineBlue = new RotatedBoxesBlue();
            camera.setPipeline(pipelineBlue);
        } else if ("Red".equalsIgnoreCase(selectedPipeline)) {
            pipelineRed = new RotatedBoxesRed();
            camera.setPipeline(pipelineRed);
        } else if ("Yellow".equalsIgnoreCase(selectedPipeline)) {
            pipelineYellow = new RotatedBoxesYellow();
            camera.setPipeline(pipelineYellow);
        } else {
            telemetry.addLine("Invalid pipeline selection. Defaulting to Yellow.");
            pipelineYellow = new RotatedBoxesYellow();
            camera.setPipeline(pipelineYellow);
        }

        // Start streaming to the dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera, 60);

        // Define the poses of the robot
        Pose2d startPose = new Pose2d(-56, -54, Math.toRadians(90));
        Pose2d parkPose = new Pose2d(-22, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence parkBySubmersible = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(parkPose) // park by submersible
                .build();

        waitForStart();
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
        if (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(parkBySubmersible);

            // Align to the closest game piece after parking
            if (pipelineBlue != null) {
                alignToPiece(drive, pipelineBlue);
            } else if (pipelineRed != null) {
                alignToPiece(drive, pipelineRed);
            } else if (pipelineYellow != null) {
                alignToPiece(drive, pipelineYellow);
            }

            // Stop the camera
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////
    private void alignToPiece(SampleMecanumDrive drive, Object pipeline) {
        double imageCenterX = 320; // Assuming 640px wide image, center is 320px
        double tolerance = X_TOLERANCE; // Acceptable deviation from center
        double power = ADJUSTMENT_POWER; // Speed for fine adjustments

        while (opModeIsActive()) {
            List<Point> rectangleCenters = new ArrayList<>(); // List of centers

            // Dynamically get the list of rectangle centers (cX values) from the pipeline
            if (pipeline instanceof RotatedBoxesBlue) {
                rectangleCenters = ((RotatedBoxesBlue) pipeline).rectangleCenters;
            } else if (pipeline instanceof RotatedBoxesRed) {
                rectangleCenters = ((RotatedBoxesRed) pipeline).rectangleCenters;
            } else if (pipeline instanceof RotatedBoxesYellow) {
                rectangleCenters = ((RotatedBoxesYellow) pipeline).rectangleCenters;
            }

            if (rectangleCenters == null || rectangleCenters.isEmpty()) {
                telemetry.addLine("No objects detected");
                telemetry.update();
                continue;
            }

            // Find the closest cX value to the center
            double closestCX = rectangleCenters.get(0).x; // Initial closest cX
            double closestDistance = Math.abs(closestCX - imageCenterX);

            for (Point center : rectangleCenters) {
                double distance = Math.abs(center.x - imageCenterX);
                if (distance < closestDistance) {
                    closestCX = center.x;
                    closestDistance = distance;
                }
            }

            telemetry.addData("Closest cX", closestCX);
            telemetry.addData("Target", imageCenterX);

            // Check if the closest cX is within the tolerance
            if (closestDistance <= tolerance) {
                telemetry.addLine("Aligned: closest cX centered");
                telemetry.update();
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0)); // Stop the robot
                break;
            } else if (closestCX < imageCenterX) {
                // Object is to the left, strafe right
                drive.setWeightedDrivePower(new Pose2d(0, power, 0));
            } else {
                // Object is to the right, strafe left
                drive.setWeightedDrivePower(new Pose2d(0, -power, 0));
            }

            drive.update();
            telemetry.update();
        }
    }
}
