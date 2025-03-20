package org.firstinspires.ftc.teamcode.developmentOpModes;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.vision.RotatedBoxesYellow;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "AutoStrafeToAlign3")
public class AutoStrafeToAlign3 extends LinearOpMode {

    private OpenCvWebcam camera;
    private RotatedBoxesYellow pipeline = new RotatedBoxesYellow();
    private Servo pivot = null;
    private static final double pivot_NEUTRAL = 0.5;
    public static double MAX_STRAFE_POWER = 0.5; // Adjustable in FTC Dashboard
    public static double MIN_STRAFE_POWER = 0.1; // Minimum power to ensure movement
    public static double PIXELS_TO_INCHES = 0.03; // Adjustable in FTC Dashboard
    public static int TOLERANCE = 25; // Adjustable in FTC Dashboard

    // Roll range constants
    private final double ROLL_MIN = 0;
    private final double ROLL_MAX = 1;

    @Override
    public void runOpMode() {
        // Initialize the FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Get the camera ID from the app resources
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        pivot = hardwareMap.get(Servo.class, "pivot");

        // Initialize the camera and pipeline
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "controlHubCam"), cameraMonitorViewId);
        pipeline = new RotatedBoxesYellow();

        // Set the pipeline and open the camera
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT, OpenCvWebcam.StreamFormat.MJPEG);
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

        waitForStart();

        while (opModeIsActive()) {
            int xOffset = 0;
            double angle = 0;
            if (pipeline.rectangleCenters.size() > 0 && pipeline.rectangleCenters.get(0) != null) {
                xOffset = (int) (pipeline.rectangleCenters.get(0).x - RotatedBoxesYellow.xTarget);
                if (!pipeline.contoursByArea2f.empty()) {
                    RotatedRect rect = Imgproc.minAreaRect(pipeline.contoursByArea2f);
                    angle = rect.angle;
                } else {angle = 90}
            }

            // Map angle (-90 to 90) to servo position (0 to 1)
            private double pivotPosition(double angle){
                return (angle+90) / 180.0 * ( ROLL_MAX - ROLL_MIN) + ROLL_MIN - 0.5;
            }
            pivot.setPosition(pivotPosition);

            telemetry.addData("xOffset", xOffset);
            telemetry.addData("MAX_STRAFE_POWER", MAX_STRAFE_POWER);
            telemetry.addData("MIN_STRAFE_POWER", MIN_STRAFE_POWER);
            telemetry.addData("PIXELS_TO_INCHES", PIXELS_TO_INCHES);
            telemetry.addData("TOLERANCE", TOLERANCE);
            telemetry.addData("Object Angle", angle);
            telemetry.addData("Pivot Position", pivotPosition);
            telemetry.update();

            if (Math.abs(xOffset) > TOLERANCE) {
                double normalizedOffset = Math.abs(xOffset) / 240.0; // Normalize based on half of 480px width
                double strafePower = MIN_STRAFE_POWER + (MAX_STRAFE_POWER - MIN_STRAFE_POWER) * normalizedOffset;
                strafePower = Math.min(strafePower, MAX_STRAFE_POWER); // Cap power at max
                strafePower *= Math.signum(xOffset); // Apply direction

                drive.setDrivePower(new Pose2d(0, -strafePower, 0));  // Apply power dynamically
            } else {
                drive.setDrivePower(new Pose2d(0, 0, 0)); // Stop when aligned
            }

            sleep(10); // Small delay to prevent overloading the loop
        }
    }
}
