package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.constants.Constants;
import org.firstinspires.ftc.teamcode.common.constants.ServoConstants;

public class Holder extends SubsystemBase {
    private final Telemetry telemetry;
    public Servo leftHolder, rightHolder;
    public AnalogInput potentiometer;
    private HoldState state;

    public enum HoldState {
        HOLDING,
        OPEN,
        PREPARE;
    }

    public Holder(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.leftHolder = hardwareMap.get(Servo.class, "leftHolder");
        this.rightHolder = hardwareMap.get(Servo.class, "rightHolder");
    }

    public void hold(){
        state = state == HoldState.OPEN ? HoldState.PREPARE: HoldState.HOLDING;
        if(state == HoldState.PREPARE){
            leftHolder.setPosition(ServoConstants.LEFTHOLDER_PREPARE.value);
            rightHolder.setPosition(ServoConstants.RIGHTHOLDER_PREPARE.value);
        }
        else{
            leftHolder.setPosition(ServoConstants.LEFTHOLDER_HOLD.value);
            rightHolder.setPosition(ServoConstants.RIGHTHOLDER_HOLD.value);
        }
    }

    public void open(){
        state = HoldState.OPEN;
        leftHolder.setPosition(ServoConstants.LEFTHOLDER_OPEN.value);
        rightHolder.setPosition(ServoConstants.RIGHTHOLDER_OPEN.value);
    }

    public void hold_Dual(){
        leftHolder.setPosition(ServoConstants.LEFTHOLDER_HOLD.value);
        rightHolder.setPosition(ServoConstants.RIGHTHOLDER_HOLD.value);
    }

    public void prepare(){
        leftHolder.setPosition(ServoConstants.LEFTHOLDER_PREPARE.value);
        rightHolder.setPosition(ServoConstants.RIGHTHOLDER_PREPARE.value);
    }

}
