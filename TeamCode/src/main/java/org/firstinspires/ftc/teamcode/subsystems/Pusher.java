package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.constants.Constants;

public class Pusher extends SubsystemBase {
    private final Telemetry telemetry;
    public Servo pushLeft, pushRight;
    public AnalogInput potentiometer;
    private double targetAngle;
    private PushState currentState = PushState.ACCELERATE;
    public boolean manualControl = false;
    private final double tolerance = 5;


    public enum PushState {
        SHOOT(70),
        ACCELERATE(55),
        PUSH(125);

        public final double angle;
        PushState(double angle) {
            this.angle = angle;
        }
    }

    public Pusher(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.pushLeft = hardwareMap.get(Servo.class, "pushLeft");
        this.pushRight = hardwareMap.get(Servo.class, "pushRight");
        this.potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        this.pushRight.setDirection(Servo.Direction.REVERSE);
        this.targetAngle = currentState.angle;
    }

    public void manualControl(double power) {
        manualControl = true;

        double servoPower = Range.clip(0.5 + power, 0, 1);
        pushLeft.setPosition(servoPower);
        pushRight.setPosition(servoPower);

        targetAngle = potentiometer.getVoltage() * Constants.VOLT_TO_DEG.value;
    }

    public void disableManual() {
        manualControl = false;
    }

    public void push() {
        currentState = PushState.PUSH;
        targetAngle = currentState.angle;
    }

    public void accelerate(){
        currentState = PushState.ACCELERATE;
        targetAngle = currentState.angle;
    }

    public void shoot(){
        currentState = PushState.SHOOT;
        targetAngle = currentState.angle;
    }

    public void periodic() {
        if (!manualControl) {
            double currentAngle = potentiometer.getVoltage() * Constants.VOLT_TO_DEG.value;
            double error = targetAngle - currentAngle;

            double servoPower = Math.abs(error) < tolerance ? 0.5 : (error > 0 ? 1 : 0);

            pushLeft.setPosition(servoPower);
            pushRight.setPosition(servoPower);

            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("State", currentState);
        }
        telemetry.update();
    }

}
