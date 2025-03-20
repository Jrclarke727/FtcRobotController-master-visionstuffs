package org.firstinspires.ftc.teamcode.developmentOpModes;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.vision.RotatedBoxesYellow;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
@Autonomous(name = "AutoStrafeToAlign")
public class AutoStrafeToAlign extends LinearOpMode {

    private OpenCvWebcam camera;
    private RotatedBoxesYellow pipeline = new RotatedBoxesYellow();

    public static double STRAFE_POWER = 0.1; // Adjustable in FTC Dashboard
    public static double PIXELS_TO_INCHES = 0.03; // Adjustable in FTC Dashboard
    public static int TOLERANCE = 8; // Adjustable in FTC Dashboard

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
            if (!pipeline.rectangleCenters.isEmpty() && pipeline.rectangleCenters.get(0) != null) {
                xOffset = (int) (pipeline.rectangleCenters.get(0).x - RotatedBoxesYellow.xTarget);
            }

            telemetry.addData("xOffset", xOffset);
            telemetry.addData("STRAFE_POWER", STRAFE_POWER);
            telemetry.addData("PIXELS_TO_INCHES", PIXELS_TO_INCHES);
            telemetry.addData("TOLERANCE", TOLERANCE);
            telemetry.update();

            if (Math.abs(xOffset) > TOLERANCE) {
                double strafeDistance = xOffset * PIXELS_TO_INCHES;
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeRight(strafeDistance)
                        .build());
            }
        }
    }
}
