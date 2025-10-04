package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="on bot java version", group="Linear OpMode")

public class OnBotJavaVersion extends LinearOpMode {

    private DcMotorEx backShooter;
    private DcMotorEx frontShooter;
    private DcMotor preShooter = null;

    // 你期望的飞轮速度（单位：RPM）
    private double targetRPM = 1800.0;

    @Override
    public void runOpMode() {
        backShooter  = (DcMotorEx) hardwareMap.get(DcMotor.class, "backShooter");
        frontShooter = (DcMotorEx) hardwareMap.get(DcMotor.class, "frontShooter");
        preShooter = hardwareMap.get(DcMotor.class, "preShooter");
        // 对装：让两电机最终带动飞轮同向转动
        frontShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // 飞轮通常采用滑行更合适
        backShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        backShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 使用编码器的速度控制
        backShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 如需进一步稳定速度，可之后再调 PIDF（先用默认就好）
        // backShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        // frontShooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        telemetry.addLine("Press & hold A to spin at target RPM");
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // // 将 RPM 转为 ticks/s（SDK 的 setVelocity 默认单位是 ticks/s）
            // double tpsL = rpmToTicksPerSec(targetRPM, backShooter.getMotorType().getTicksPerRev());
            // double tpsR = rpmToTicksPerSec(targetRPM, frontShooter.getMotorType().getTicksPerRev());

            if (gamepad1.a) {
                // 按住 A — 按目标转速运行
                backShooter.setVelocity(targetRPM);
                frontShooter.setVelocity(targetRPM);
            } else {
                // 松开 A — 停止
                backShooter.setVelocity(0);
                frontShooter.setVelocity(0);
            }

            if(gamepad1.b){
                preShooter.setPower(-0.7);

            }else{
                preShooter.setPower(0);
            }

            telemetry.addData("A pressed", gamepad1.a);
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("L vel (tps)", "%.0f", backShooter.getVelocity());
            telemetry.addData("R vel (tps)", "%.0f", frontShooter.getVelocity());
            telemetry.update();

        }

    }
}
