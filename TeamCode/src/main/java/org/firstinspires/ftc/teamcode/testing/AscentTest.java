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


@TeleOp(name = "Ascent Test", group = "test")
@Config
public class AscentTest extends LinearOpMode {

    private final Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    public static int encoder_position = 41311;
    public static double max_power = 1;
    public static boolean read_only = false;


    @Override
    public void runOpMode() {
        DcMotorEx motor0 = hardwareMap.get(DcMotorEx.class, "ascent");
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if(read_only){
                motor0.setPower(1);
                motor0.setTargetPosition(encoder_position);
                motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motor0.setPower(max_power);
            }
            else{
                motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor0.setPower(max_power * (-gamepad1.left_stick_y));
            }
            telemetry_M.addData("encoder", motor0.getCurrentPosition());
            telemetry_M.addData("power", motor0.getPower());
            telemetry_M.addData("velocity", motor0.getVelocity());
            telemetry_M.update();
        }
    }
}