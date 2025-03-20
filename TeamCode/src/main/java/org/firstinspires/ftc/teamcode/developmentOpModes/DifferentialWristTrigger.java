package org.firstinspires.ftc.teamcode.developmentOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DifferentialWristTrigger", group = "TeleOp")
public class DifferentialWristTrigger extends OpMode {

    // Servo variables for the differential mechanism
    Servo servoL;
    Servo servoR;

    // Variables to track servo positions
    double servoLPosition = 0.5; // Starting at the midpoint
    double servoRPosition = 0.5; // Starting at the midpoint

    // Increment value for adjustments
    final double INCREMENT = 0.006; // Adjust as needed for sensitivity
    final double ROLL_SCALE = 0.25; // Roll axis moves through 25% range
    final double ROLL_MIN = 0.375; // Minimum position for roll (25% of range)
    final double ROLL_MAX = 0.625; // Maximum position for roll (25% of range)

    // Pitch limits
    final double PITCH_MIN = 0.15;
    final double PITCH_MAX = 0.85;

    // Separate roll offset for L2 control
    double rollOffset = 0.0;

    @Override
    public void init() {
        // Initialize servos
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");

        // Set initial servo positions
        servoL.setPosition(servoLPosition);
        servoR.setPosition(servoRPosition);

        // Telemetry initialization
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Adjust pitch
        if (gamepad1.dpad_up) {
            servoLPosition -= INCREMENT;
            servoRPosition += INCREMENT;
        } else if (gamepad1.dpad_down) {
            servoLPosition += INCREMENT;
            servoRPosition -= INCREMENT;
        }

        // Clamp pitch positions to PITCH_MIN and PITCH_MAX
        servoLPosition = Math.min(Math.max(servoLPosition, PITCH_MIN), PITCH_MAX);
        servoRPosition = Math.min(Math.max(servoRPosition, PITCH_MIN), PITCH_MAX);

        // Adjust roll based on L2 or D-pad
        if (gamepad1.left_trigger > 0) {
            rollOffset = (gamepad1.left_trigger * (ROLL_MAX - ROLL_MIN)) + ROLL_MIN - 0.5;
        } else if (gamepad1.dpad_right) {
            rollOffset -= INCREMENT * ROLL_SCALE;
        } else if (gamepad1.dpad_left) {
            rollOffset += INCREMENT * ROLL_SCALE;
        } else {
            rollOffset = 0.0;
        }

        // Clamp rollOffset to valid roll range
        rollOffset = Math.min(Math.max(rollOffset, ROLL_MIN - 0.5), ROLL_MAX - 0.5);

        // Combine pitch and roll adjustments
        double adjustedServoLPosition = servoLPosition + rollOffset;
        double adjustedServoRPosition = servoRPosition + rollOffset;

        // Clamp combined positions to valid range [0.0, 1.0]
        adjustedServoLPosition = Math.min(Math.max(adjustedServoLPosition, 0.0), 1.0);
        adjustedServoRPosition = Math.min(Math.max(adjustedServoRPosition, 0.0), 1.0);

        // Set servo positions
        servoL.setPosition(adjustedServoLPosition);
        servoR.setPosition(adjustedServoRPosition);

        // Telemetry data for debugging and monitoring
        telemetry.addData("ServoL Position", adjustedServoLPosition);
        telemetry.addData("ServoR Position", adjustedServoRPosition);
        telemetry.addData("Pitch Offset", servoLPosition - 0.5);
        telemetry.addData("Roll Offset", rollOffset);
        telemetry.update();
    }
}
