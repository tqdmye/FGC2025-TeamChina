package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Test Shooter PID")
@Config
public class ShooterTest extends LinearOpMode {
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static double setP = 0.5;
    public static double setI = 0.0001;
    public static double setD = 0.1;
    public static double setPower = 0.78;
    public static boolean isPIDControl = true;
    public static int shooterMinVelocity = 1600;

    @Override
    public void runOpMode() throws InterruptedException {
        Motor leftShooter = new Motor(hardwareMap, "leftShooter", 28, 6000);
        Motor rightShooter = new Motor(hardwareMap, "rightShooter", 28, 6000);
        Motor preShooter = new Motor(hardwareMap, "preShooter", 28, 6000);
        Motor blender = new Motor(hardwareMap, "blender", 28, 6000);

        leftShooter.setInverted(false);
        rightShooter.setInverted(true);
        preShooter.setInverted(true);
        blender.setInverted(true);

        leftShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        preShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftShooter.resetEncoder();
        rightShooter.resetEncoder();
        preShooter.resetEncoder();

        preShooter.setRunMode(Motor.RunMode.RawPower);


        if(isPIDControl) {
            leftShooter.setRunMode(Motor.RunMode.VelocityControl);
            rightShooter.setRunMode(Motor.RunMode.VelocityControl);
            leftShooter.setVeloCoefficients(setP, setI, setD);
            rightShooter.setVeloCoefficients(setP, setI, setD);
        }
        else {
            leftShooter.setRunMode(Motor.RunMode.RawPower);
            rightShooter.setRunMode(Motor.RunMode.RawPower);
        }

        waitForStart();
        while (opModeIsActive()) {
            leftShooter.set(setPower);
            rightShooter.set(setPower);

            if(leftShooter.encoder.getRawVelocity() > shooterMinVelocity){
                preShooter.set(1);
                blender.set(1);
            }

            if(leftShooter.encoder.getRawVelocity() < shooterMinVelocity){
//                preShooter.set(0);
                blender.set(0);
            }

            telemetry_M.addData("Acceleration", leftShooter.encoder.getAcceleration());
            telemetry_M.addData("CorrectedVelocity", leftShooter.encoder.getCorrectedVelocity());
            telemetry_M.addData("Velocity", leftShooter.encoder.getRawVelocity());

            telemetry_M.update();
        }

    }
}
