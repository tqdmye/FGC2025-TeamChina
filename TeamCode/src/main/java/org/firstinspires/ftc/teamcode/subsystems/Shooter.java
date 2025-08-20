package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.constants.OtherConstants;

public class Shooter{
    private Motor leftShooter;
    private Motor rightShooter;
    private Motor preShooter;
    private Motor blender;
    private Motor intake;

    public Shooter (final HardwareMap hardwareMap) {
        this.leftShooter = new Motor(hardwareMap, "leftShooter", 28, 6000);
        this.rightShooter = new Motor(hardwareMap, "rightShooter", 28, 6000);
        this.preShooter = new Motor(hardwareMap, "preShooter", 28, 6000);
        this.blender = new Motor(hardwareMap, "blender", 28, 6000);
        this.intake = new Motor(hardwareMap, "intake",28,6000);

        leftShooter.setInverted(false);
        rightShooter.setInverted(true);
        preShooter.setInverted(true);
        blender.setInverted(true);

        leftShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        preShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        leftShooter.resetEncoder();
        rightShooter.resetEncoder();
        preShooter.resetEncoder();

        preShooter.setRunMode(Motor.RunMode.RawPower);

        leftShooter.setRunMode(Motor.RunMode.VelocityControl);
        rightShooter.setRunMode(Motor.RunMode.VelocityControl);
        leftShooter.setVeloCoefficients(OtherConstants.SHOOTER_P.value, OtherConstants.SHOOTER_I.value, OtherConstants.SHOOTER_D.value);
        rightShooter.setVeloCoefficients(OtherConstants.SHOOTER_P.value, OtherConstants.SHOOTER_I.value, OtherConstants.SHOOTER_D.value);

        leftShooter.set(OtherConstants.SHOOTER_PREPARE_POWER.value); /***/
    }

    public void startShoot(Telemetry telemetry){
        leftShooter.set(OtherConstants.SHOOTER_POWER.value);
        rightShooter.set(OtherConstants.SHOOTER_POWER.value);

        if(leftShooter.encoder.getRawVelocity() > OtherConstants.SHOOTER_MIN_VELOCITY.value){
            preShooter.set(OtherConstants.PRESHOOTER_POWER.value);
            blender.set(1);
            intake.set(0.8);
        }

        if(leftShooter.encoder.getRawVelocity() < OtherConstants.SHOOTER_MIN_VELOCITY.value){
//                preShooter.set(0);
            blender.set(0);
            intake.set(0);
        }

        telemetry.addData("Acceleration", leftShooter.encoder.getAcceleration());
        telemetry.addData("CorrectedVelocity", leftShooter.encoder.getCorrectedVelocity());
        telemetry.addData("Shooter Velocity", leftShooter.encoder.getRawVelocity());
        telemetry.addData("PreShooter Velocity", preShooter.encoder.getRawVelocity());

        telemetry.update();
    }

    public void stopShoot(Telemetry telemetry){
        leftShooter.set(OtherConstants.SHOOTER_PREPARE_POWER.value);
        rightShooter.set(OtherConstants.SHOOTER_PREPARE_POWER.value);

        if(leftShooter.encoder.getRawVelocity() < OtherConstants.SHOOTER_MIN_VELOCITY.value){
//                preShooter.set(0);
            blender.set(0);
            intake.set(0);
        }

        telemetry.addData("Acceleration", leftShooter.encoder.getAcceleration());
        telemetry.addData("CorrectedVelocity", leftShooter.encoder.getCorrectedVelocity());
        telemetry.addData("Shooter Velocity", leftShooter.encoder.getRawVelocity());
        telemetry.addData("PreShooter Velocity", preShooter.encoder.getRawVelocity());

        telemetry.update();
    }

}
