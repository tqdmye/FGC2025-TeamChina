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
  public Servo leftPush, rightPush;
  public AnalogInput potentiometer;
  public double targetAngle, currentAngle;
  public PushState currentState = PushState.ACCELERATE;
  public boolean manualControl = false;
  private final double tolerance = 4;

  public enum PushState {
    SHOOT(61),
    ACCELERATE(55.5),
    PUSH(125);

    public final double angle;

    PushState(double angle) {
      this.angle = angle;
    }
  }

  public Pusher(Telemetry telemetry, HardwareMap hardwareMap) {
    this.telemetry = telemetry;
    this.leftPush = hardwareMap.get(Servo.class, "leftPush");
    this.rightPush = hardwareMap.get(Servo.class, "rightPush");
    this.potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
    this.rightPush.setDirection(Servo.Direction.REVERSE);
    this.targetAngle = currentState.angle;
  }

  public void manualControl(double power) {
    if (power == 0 && manualControl) {
      leftPush.setPosition(0.5);
      rightPush.setPosition(0.5);
      return;
    } else if (power == 0) return;

    manualControl = true;
    if (potentiometer.getVoltage() * Constants.VOLT_TO_DEG.value > 150 && power > 0) {
      power = 0;
    }
    if (potentiometer.getVoltage() * Constants.VOLT_TO_DEG.value < 60 && power < 0) {
      power = 0;
    }
    double servoPower = Range.clip(0.5 + power, 0, 1);
    leftPush.setPosition(servoPower);
    rightPush.setPosition(servoPower);

    targetAngle = potentiometer.getVoltage() * Constants.VOLT_TO_DEG.value;
  }

  public void disableManual() {
    manualControl = false;
  }

  public void push() {
    disableManual();
    currentState = PushState.PUSH;
    targetAngle = currentState.angle;
  }

  public void accelerate() {
    disableManual();
    currentState = PushState.ACCELERATE;
    targetAngle = currentState.angle;
  }

  public void shoot() {
    disableManual();
    currentState = PushState.SHOOT;
    targetAngle = currentState.angle;
  }

  public void periodic() {
    if (!manualControl) {
      currentAngle = potentiometer.getVoltage() * Constants.VOLT_TO_DEG.value;
      double error = targetAngle - currentAngle;

      double servoPower = Math.abs(error) < tolerance ? 0.5 : (error > 0 ? 1 : 0);

      leftPush.setPosition(servoPower);
      rightPush.setPosition(servoPower);
    }
  }
}
