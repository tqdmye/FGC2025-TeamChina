package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.constants.ServoConstants;

public class Ascent extends SubsystemBase {
  private DcMotorEx ascent;
  private Servo brakeAscent;

  public Ascent(final HardwareMap hardwareMap) {
    ascent = hardwareMap.get(DcMotorEx.class, "ascent");
    brakeAscent = hardwareMap.get(Servo.class, "brakeAscent");

    ascent.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ascent.setDirection(DcMotorSimple.Direction.REVERSE);
    ascent.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  public void ascentUp() {
    ascent.setTargetPosition(ascent.getCurrentPosition() + 500);
    ascent.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ascent.setPower(1);
  }

  public void ascentDown() {
    ascent.setTargetPosition(ascent.getCurrentPosition() - 100);
    ascent.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ascent.setPower(1);
    brakeAscent.setPosition(ServoConstants.BRAKE_ASCENT_UNLOCK.value);
  }

  public void brakeAscent() {
    brakeAscent.setPosition(ServoConstants.BRAKE_ASCENT_LOCK.value);
  }

  public void unlockAscentDual() {
    brakeAscent.setPosition(ServoConstants.BRAKE_ASCENT_UNLOCK.value);
  }

  public void ascentUpDual(double power) {
    ascent.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    if (power < 0) {
      power = power * 0.5;
    }
    ascent.setPower(power);
  }
}
