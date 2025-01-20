package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TowerControl", group = "Testing")
public class TowerControl extends LinearOpMode {

    private DcMotorEx towerMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motor
        towerMotor = hardwareMap.get(DcMotorEx.class, "tower");

        // Reset encoder and set motor mode
        towerMotor.setDirection(DcMotor.Direction.REVERSE);
        towerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the driver to press start
        waitForStart();

        while (opModeIsActive()) {
            // Raise the motor when the gamepad1 'A' button is pressed
            if (gamepad1.a) {
                towerMotor.setPower(1); // Move up
            }
            // Lower the motor when the gamepad1 'B' button is pressed
            else if (gamepad1.b) {
                towerMotor.setPower(-1); // Move down
            }
            // Stop the motor when no button is pressed
            else {
                towerMotor.setPower(0);
            }

            // Display the encoder position on telemetry
            telemetry.addData("Tower Encoder Position", towerMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
