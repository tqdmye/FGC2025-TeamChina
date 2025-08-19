package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.constants.MotorConstants;

public class Ascent extends SubsystemBase {
    private DcMotorEx ascent;

    public Ascent(final HardwareMap hardwareMap) {
        ascent = hardwareMap.get(DcMotorEx.class, "ascent");
        ascent.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascent.setDirection(DcMotorSimple.Direction.REVERSE);
        ascent.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void ascentCloseloop(){
        ascent.setPower(1);
        ascent.setTargetPosition(MotorConstants.ASCENT.value);
        ascent.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ascentOpenloop(){
        ascent.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascent.setPower(1);
    }
    public void ascentOpenloopDown(){
        ascent.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascent.setPower(-1);
    }
    public void reset(){
        ascent.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ascent.setPower(0);
        ascent.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
