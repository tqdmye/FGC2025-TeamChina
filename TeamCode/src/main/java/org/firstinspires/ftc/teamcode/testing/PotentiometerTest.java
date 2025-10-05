package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test Potentiometer", group = "test")
public class PotentiometerTest extends LinearOpMode {
    // Define variables for our potentiometer and motor
    AnalogInput potentiometer;
    Servo pushLeft, pushRight;

    // Define variable for the current voltage
    double currentVoltage;

    @Override
    public void runOpMode() {
        // Get the potentiometer and motor from hardwareMap
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        pushLeft = hardwareMap.get(Servo.class, "pushLeft");
        pushRight = hardwareMap.get(Servo.class, "pushRight");

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // Update currentVoltage from the potentiometer
            currentVoltage = potentiometer.getVoltage();

            // Turn the motor on or off based on the potentiometer’s position
            if (currentVoltage < 1.65) {
                pushLeft.setPosition(0.5);
                pushRight.setPosition(0.5);
            } else {
                pushLeft.setPosition(0);
                pushRight.setPosition(1);
            }

            // Show the potentiometer’s voltage in telemetry
            telemetry.addData("Potentiometer voltage", currentVoltage);
            telemetry.addData("Potentiometer angle", currentVoltage*81.8);
            telemetry.addData("Left Servo", pushLeft.getPosition());
            telemetry.addData("Right Servo", pushRight.getPosition());
            telemetry.update();
        }
    }
}