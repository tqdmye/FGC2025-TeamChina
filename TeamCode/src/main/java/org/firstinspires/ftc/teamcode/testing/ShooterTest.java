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


@TeleOp(name = "Test Shooter PID", group = "test")
@Config
public class ShooterTest extends LinearOpMode {
    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static double setP = 0.1;
    public static double setI = 0;
    public static double setD = 0.01;
    public static double setF = 1;
    //    public static double setShooterPower = 0.8;
    public static double setPreShooterPower = 0.8;
    public static boolean isPIDControl = true;
    public static double shooterMinVelocity = 1040.0;
    public static double frontShooterVelocity = 1420.0;
    public static double backShooterVelocity = 2280.0;

//    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
//    static final double     DRIVE_GEAR_REDUCTION    = 30.24;
//    static final double     WHEEL_CIRCUMFERENCE_MM  = 90.0 * 3.14;
//
//    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
//    static final double     COUNTS_PER_MM           = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        DcMotorEx frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        DcMotorEx preShooter = hardwareMap.get(DcMotorEx.class, "preShooter");
        DcMotorEx blender = hardwareMap.get(DcMotorEx.class, "blender");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

//        backShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        double TPS = (175/ 60) * COUNTS_PER_WHEEL_REV;

        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        preShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        blender.setDirection(DcMotorSimple.Direction.REVERSE);

        backShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        frontShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        preShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        backShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

//        backShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontShooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        backShooter.setVelocity(TPS);
//        frontShooter.setVelocity(TPS);


        while (opModeIsActive()) {
            backShooter.setVelocity(backShooterVelocity);
            frontShooter.setVelocity(frontShooterVelocity);

            if(frontShooter.getVelocity() > shooterMinVelocity){
//            if(gamepad1.a){
                preShooter.setPower(setPreShooterPower);
                blender.setPower(1);
                intake.setPower(1);
            }

            if(frontShooter.getVelocity() < shooterMinVelocity){
//            if(gamepad1.b){
                preShooter.setPower(0);
                blender.setPower(0);
                intake.setPower(0);
            }

            telemetry_M.addData("Front Shooter Velocity", frontShooter.getVelocity());
            telemetry_M.addData("Back Shooter Velocity", backShooter.getVelocity());
            telemetry_M.addData("PreShooter Velocity", preShooter.getVelocity());
            telemetry_M.update();
        }

    }
}
