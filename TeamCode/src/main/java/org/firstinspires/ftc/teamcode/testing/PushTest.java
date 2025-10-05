package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.common.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;

@TeleOp(name = "Test Push", group = "test")
public class PushTest extends CommandOpMode {
    private Pusher pusher;
    private GamepadEx gamepadEx1;


    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();

        pusher = new Pusher(telemetry, hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(()->pusher.push()));
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()->pusher.accelerate()));
        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new InstantCommand(()->pusher.shoot()));
    }

    @Override
    public void run() {
        double y = -gamepad1.right_stick_y;

        if (Math.abs(y) > 0.05) {
            if (pusher.potentiometer.getVoltage() * Constants.VOLT_TO_DEG.value > 150
                    && y > 0) {
                y = 0;
            }
            if (pusher.potentiometer.getVoltage() * Constants.VOLT_TO_DEG.value < 60
                    && y < 0) {
                y = 0;
            }
            pusher.manualControl(y * 0.5);
        } else if (pusher.manualControl) {
            pusher.disableManual();
        }

        CommandScheduler.getInstance().run();
    }

}
