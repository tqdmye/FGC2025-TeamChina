package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.common.constants.Constants;
import org.firstinspires.ftc.teamcode.common.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.subsystems.Ascent;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Pusher;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.TankDrive;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name = "FGC TeleOp Auto shoot")
public class FGCTeleOp_Auto extends CommandOpMode {
    private TriggerReader triggerReader;
    private SlewRateLimiter driverLimiter;
    private SlewRateLimiter turnLimiter;

//    private List<LynxModule> allHubs;

    private TankDrive tankDrive;
    private Ascent ascent;
    private Vision vision;
    private Shooter shooter;
    private Pusher pusher;
    private GamepadEx gamepadEx1, gamepadEx2;
    public State state;


    public enum State{
        FREE,
        VISION,
        SHOOT
    }


    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CommandScheduler.getInstance().reset();

//        for(LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        //Subsystems Initialization
        tankDrive = new TankDrive(hardwareMap);
        ascent = new Ascent(hardwareMap);
        vision = new Vision(telemetry, hardwareMap);
        shooter = new Shooter(hardwareMap);
        pusher = new Pusher(telemetry, hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);

        driverLimiter = new SlewRateLimiter(4);
        turnLimiter = new SlewRateLimiter(3);

        state = State.FREE;

        tankDrive.setDefaultCommand(
                new TankDriveCommand(
                        tankDrive,
                        () -> -driverLimiter.calculate(gamepadEx1.getLeftY()) ,
                        () -> gamepadEx1.getRightX(),
                        () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5));

        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()->ascent.ascentCloseloop()));

        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(()->ascent.reset()));

        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(()-> ascent.ascentOpenloop()));

        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(()-> ascent.ascentOpenloopDown()));

//        gamepadEx1
//                .getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(
//                        new ConditionalCommand(
//                                new InstantCommand(()->this.state=State.FREE),
//                                new InstantCommand(()->this.state=State.VISION),
//                                ()->this.state == State.VISION));

        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(() -> vision.driveWithVision(tankDrive, telemetry, true)));

        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new ShooterCommand(
                        shooter,
                        Constants.SHOOTER_VELOCITY.value,
                        Constants.SHOOTER_MIN_VELOCITY.value
                ));
        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(()->shooter.counterPreShooter()));

        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(()->shooter.intake()));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(()->pusher.push()));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(()->pusher.accelerate()));

        gamepadEx1
                .getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(new InstantCommand(()->pusher.shoot()));

//        new RunCommand(pusher::push, pusher).schedule();
//
//        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(pusher::nextState);
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
        updateTelemetry(telemetry);
    }
}
