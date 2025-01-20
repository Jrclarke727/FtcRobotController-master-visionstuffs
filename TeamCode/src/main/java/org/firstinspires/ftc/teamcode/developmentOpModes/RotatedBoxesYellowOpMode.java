package org.firstinspires.ftc.teamcode.developmentOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.RotatedBoxesYellow;

@Autonomous(name = "RotatedBoxesYellowOpMode")
public class RotatedBoxesYellowOpMode extends LinearOpMode {

    private OpenCvCamera camera;
    private RotatedBoxesYellow pipeline;

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
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(camera, 60); // Stream to FTC Dashboard
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error code: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Display data from the pipeline to the telemetry
            telemetry.addData("Center X", pipeline.cX);
            telemetry.addData("Center Y", pipeline.cY);
            telemetry.addData("Width", pipeline.width);
            telemetry.update();

            // Allow OpenCV processing to continue
            sleep(50);
        }

        // Stop the camera when the OpMode stops
        camera.stopStreaming();
        camera.closeCameraDevice();
    }
}
