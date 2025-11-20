package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.constants.Constants;

public class Shooter extends SubsystemBase {
    public DcMotorEx backShooter, frontShooter, preShooter, blender, intake;
    public Telemetry telemetry;
    public ShooterState state = ShooterState.FREE;

    public enum ShooterState {
        FREE,
        SHOOTING,
        ACCELERATING,
        STOP_INTAKE,
        INTAKE,;
    }

    public Shooter(Telemetry telemetry, final HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        this.frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        this.preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");
        this.blender = hardwareMap.get(DcMotorEx.class, "blender");
        this.intake = hardwareMap.get(DcMotorEx.class, "intake");

        frontShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        backShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        preShooter.setDirection(DcMotorSimple.Direction.FORWARD);
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
//        backShooter.setVelocityPIDFCoefficients(Constants.SHOOTER_P.value, Constants.SHOOTER_I.value, Constants.SHOOTER_D.value, Constants.SHOOTER_F.value);
//        frontShooter.setVelocityPIDFCoefficients(Constants.SHOOTER_P.value, Constants.SHOOTER_I.value, Constants.SHOOTER_D.value,Constants.SHOOTER_F.value);

        backShooter.setVelocity(Constants.SHOOTER_PREPARE_VELOCITY.value);
        frontShooter.setVelocity(Constants.SHOOTER_PREPARE_VELOCITY.value);
    }

    public void setShooterVelocity(double frontVelocity, double backVelocity) {
        frontShooter.setVelocity(frontVelocity);
        backShooter.setVelocity(backVelocity);
    }

    public double getFrontShooterVelocity() {
        return frontShooter.getVelocity();
    }

    public void shooting(double power) {
        preShooter.setPower(power > 0 ? Constants.PRESHOOTER_POWER.value : 0);
        blender.setPower(power > 0 ? Constants.BLENDER_POWER.value : 0);
        if(state == ShooterState.STOP_INTAKE) return;
        intake.setPower(power > 0 ? Constants.INTAKE_POWER.value : 0);
    }

    public void counterPreShooterAndBlender() {
        preShooter.setPower(-Constants.PRESHOOTER_POWER.value);
        blender.setPower(-Constants.BLENDER_POWER.value);
    }

    public void intake() {
        preShooter.setPower(-Constants.PRESHOOTER_POWER.value);
        intake.setPower(Constants.INTAKE_POWER.value);
        blender.setPower(Constants.BLENDER_POWER.value);
        state = ShooterState.INTAKE;
    }

    public void counterIntake() {
        intake.setPower(-Constants.INTAKE_POWER.value);
        blender.setPower(-Constants.BLENDER_POWER.value);
    }

    public void reset() {
        intake.setPower(0);
        blender.setPower(0);
        preShooter.setPower(0);
    }

    public void stopIntake(){
        state = ShooterState.STOP_INTAKE;
        intake.setPower(0);
    }
}