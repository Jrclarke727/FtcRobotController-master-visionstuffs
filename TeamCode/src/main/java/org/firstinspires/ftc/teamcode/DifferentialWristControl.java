package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DifferentialWristControl", group = "TeleOp")
public class DifferentialWristControl extends OpMode {

    // Servo variables for the differential mechanism
    Servo servoL;
    Servo servoR;

    // Variables to track servo positions
    double servoLPosition = 0.5; // Starting at the midpoint
    double servoRPosition = 0.5; // Starting at the midpoint

    // Increment value for adjustments
    final double INCREMENT = 0.01; // Adjust as needed for sensitivity
    final double ROLL_SCALE = 0.75; // Roll axis moves 25% less than pitch
    final double ROLL_MIN = 0.375; // Minimum position for roll (25% of range)
    final double ROLL_MAX = 0.625; // Maximum position for roll (25% of range)

    @Override
    public void init() {
        // Initialize servos
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");

        // Set initial servo positions
        servoL.setPosition(servoLPosition);
        servoR.setPosition(servoRPosition);
    }

    @Override
    public void loop() {
        if (gamepad1.share) {
            // Reset both servos to their neutral position
            servoLPosition = 0.5;
            servoRPosition = 0.5;
        } else {
            // Adjust pitch
            if (gamepad1.dpad_up) {
                servoLPosition -= INCREMENT;
                servoRPosition += INCREMENT;
            } else if (gamepad1.dpad_down) {
                servoLPosition += INCREMENT;
                servoRPosition -= INCREMENT;
            }
        }

        // Adjust roll
        if (gamepad1.dpad_right) {
            servoLPosition -= INCREMENT * ROLL_SCALE;
            servoRPosition -= INCREMENT * ROLL_SCALE;
        } else if (gamepad1.dpad_left) {
            servoLPosition += INCREMENT * ROLL_SCALE;
            servoRPosition += INCREMENT * ROLL_SCALE;
        }

        // Clamp positions to valid range [0.0, 1.0]
        servoLPosition = Math.min(Math.max(servoLPosition, 0.0), 1.0);
        servoRPosition = Math.min(Math.max(servoRPosition, 0.0), 1.0);

        // Limit roll range to 25% of motion
        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            servoLPosition = Math.min(Math.max(servoLPosition, ROLL_MIN), ROLL_MAX);
            servoRPosition = Math.min(Math.max(servoRPosition, ROLL_MIN), ROLL_MAX);
        }

        // Set servo positions
        servoL.setPosition(servoLPosition);
        servoR.setPosition(servoRPosition);
    }
}
