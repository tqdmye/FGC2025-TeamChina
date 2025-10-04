package org.firstinspires.ftc.teamcode.testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Shooter motor test")
@Config
public class ShooterMotorsTest extends LinearOpMode {
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static double setP = 1;
    public static double setI = 0.0001;
    public static double setD = 1;
    public static double setF = 0;
    public static double setShooterPower = 0.8;
    public static double setPreShooterPower = 0.8;
    public static boolean isPIDControl = true;
    public static double shooterMinVelocity = 1640;
    public static double shooterVelocity = 1800;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        DcMotorEx frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        DcMotorEx preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");
        DcMotorEx blender = hardwareMap.get(DcMotorEx.class, "blender");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

        backShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        preShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        blender.setDirection(DcMotorSimple.Direction.REVERSE);

        backShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        preShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        blender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        preShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        if(isPIDControl) {
            backShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backShooter.setVelocityPIDFCoefficients(setP, setI, setD, setF);
            frontShooter.setVelocityPIDFCoefficients(setP, setI, setD, setF);
        }
        else {
            backShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            frontShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();
        while (opModeIsActive()) {
            if(isPIDControl){
                backShooter.setVelocity(shooterVelocity);
                frontShooter.setVelocity(shooterVelocity);
            }
            else{
                backShooter.setPower(setShooterPower);
                frontShooter.setPower(setShooterPower);
            }

            if(backShooter.getVelocity() > shooterMinVelocity || gamepad1.a){
                preShooter.setPower(setPreShooterPower);
                blender.setPower(1);
                intake.setPower(0.8);
            }
            if(backShooter.getVelocity() < shooterMinVelocity || gamepad1.b){
//                preShooter.set(0);
                blender.setPower(0);
                intake.setPower(0);
            }
            if(gamepad1.dpad_down){
                backShooter.setPower(0);
                frontShooter.setPower(0);
            }

            telemetry_M.addData("Shooter Velocity", backShooter.getVelocity());
            telemetry_M.addData("PreShooter Velocity", preShooter.getVelocity());
            telemetry_M.update();
        }
    }
}