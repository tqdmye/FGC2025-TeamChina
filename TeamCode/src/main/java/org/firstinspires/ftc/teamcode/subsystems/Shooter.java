package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.constants.OtherConstants;

public class Shooter extends SubsystemBase {
    public DcMotorEx backShooter, frontShooter, preShooter, blender, intake;

    public Shooter (final HardwareMap hardwareMap) {
        this.backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        this.frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        this.preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");
        this.blender = hardwareMap.get(DcMotorEx.class, "blender");
        this.intake = hardwareMap.get(DcMotorEx.class, "intake");

        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        preShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        blender.setDirection(DcMotorSimple.Direction.REVERSE);

        backShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        preShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        preShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backShooter.setVelocityPIDFCoefficients(OtherConstants.SHOOTER_P.value, OtherConstants.SHOOTER_I.value, OtherConstants.SHOOTER_D.value, OtherConstants.SHOOTER_F.value);
//        frontShooter.setVelocityPIDFCoefficients(OtherConstants.SHOOTER_P.value, OtherConstants.SHOOTER_I.value, OtherConstants.SHOOTER_D.value,OtherConstants.SHOOTER_F.value);

        backShooter.setVelocity(OtherConstants.SHOOTER_PREPARE_VELOCITY.value);
        frontShooter.setVelocity(OtherConstants.SHOOTER_PREPARE_VELOCITY.value);
    }


    public void setShooterVelocity(double velocity) {
        backShooter.setVelocity(OtherConstants.SHOOTER_VELOCITY.value);
        frontShooter.setVelocity(OtherConstants.SHOOTER_VELOCITY.value);
    }

    public double getShooterVelocity() {
        return backShooter.getVelocity();
    }

    public void shooting(double power) {
        preShooter.setPower(power > 0 ? OtherConstants.PRESHOOTER_POWER.value: 0);
        blender.setPower(power > 0 ? OtherConstants.BLENDER_POWER.value : 0);
        intake.setPower(power > 0 ? OtherConstants.INTAKE_POWER.value : 0);
    }

    public void stopAll() {
        backShooter.setVelocity(OtherConstants.SHOOTER_PREPARE_VELOCITY.value);
        frontShooter.setVelocity(OtherConstants.SHOOTER_PREPARE_VELOCITY.value);
        preShooter.setPower(0);
        blender.setPower(0);
        intake.setPower(0);
    }

    public void counterPreShooter(){
        preShooter.setPower(-OtherConstants.PRESHOOTER_POWER.value);
    }

    public void intake(){
        preShooter.setPower(0);
        intake.setPower(OtherConstants.INTAKE_POWER.value);
    }
}