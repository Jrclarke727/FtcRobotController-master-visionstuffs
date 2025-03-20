package org.firstinspires.ftc.teamcode.developmentOpModes;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.vision.RotatedBoxesYellow;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
@Autonomous(name = "AutoStrafeToAlign2")
public class AutoStrafeToAlign2 extends LinearOpMode {
    private Servo pivot = null;
    private static final double pivot_NEUTRAL = 0.64;


    private OpenCvWebcam camera;
    private RotatedBoxesYellow pipeline = new RotatedBoxesYellow();

    public static PIDCoefficients STRAFE_PID = new PIDCoefficients(-0.3, 0, 0.01); // Adjustable in FTC Dashboard
    public static double PIXELS_TO_INCHES = 0.03; // Adjustable in FTC Dashboard
    public static int TOLERANCE = 3; // Adjustable in FTC Dashboard
    public static double MIN_MOVE_DISTANCE = 0.4; // Prevents zero-length trajectories

    @Override
    public void runOpMode() {
        // Initialize the FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Get the camera ID from the app resources
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Initialize the camera and pipeline
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "controlHubCam"), cameraMonitorViewId);
        pipeline = new RotatedBoxesYellow();

        // Set the pipeline and open the camera
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                dashboard.startCameraStream(camera, 60); // Stream to FTC Dashboard
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error code: " + errorCode);
                telemetry.update();
            }
        });

        // Initialize Road Runner drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PIDFController pidController = new PIDFController(STRAFE_PID);

        waitForStart();

        while (opModeIsActive()) {
            int xOffset = 0;
            if (!pipeline.rectangleCenters.isEmpty() && pipeline.rectangleCenters.get(0) != null) {
                xOffset = (int) (pipeline.rectangleCenters.get(0).x - RotatedBoxesYellow.xTarget);
            } else {
                telemetry.addLine("No detected objects.");
            }

            telemetry.addData("xOffset", xOffset);
            telemetry.addData("PID kP", STRAFE_PID.kP);
            telemetry.addData("PID kI", STRAFE_PID.kI);
            telemetry.addData("PID kD", STRAFE_PID.kD);
            telemetry.addData("PIXELS_TO_INCHES", PIXELS_TO_INCHES);
            telemetry.addData("TOLERANCE", TOLERANCE);
            telemetry.update();

            // If xOffset is within tolerance, stop the robot
            if (Math.abs(xOffset) <= TOLERANCE) {
                // If within tolerance, do not generate any motion profile and stop moving
                drive.setMotorPowers(0, 0, 0, 0);
                telemetry.addData("Status", "Within tolerance, robot stopped.");
            } else {
                // Otherwise, perform strafe correction
                double strafeDistance = xOffset * PIXELS_TO_INCHES;
                double correction = pidController.update(strafeDistance);

                if (Math.abs(correction) > MIN_MOVE_DISTANCE) {
                    try {
                        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
                                new MotionState(0, 0, 0),
                                new MotionState(correction, 0, 0),
                                30, // Max velocity (adjust as needed)
                                30  // Max acceleration (adjust as needed)
                        );

                        if (profile.duration() > 0) {
                            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .strafeRight(Math.signum(correction) * Math.abs(profile.end().getX()))
                                    .build());
                        }
                    } catch (Exception e) {
                        telemetry.addData("Trajectory Error", e.getMessage());
                        telemetry.update();
                    }
                }
            }
            telemetry.update();
        }
    }
}

