//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.gamepad.TriggerReader;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.commands.TankDriveCommand;
//import org.firstinspires.ftc.teamcode.common.util.SlewRateLimiter;
//import org.firstinspires.ftc.teamcode.subsystems.Ascent;
////import org.firstinspires.ftc.teamcode.subsystems.Intake;
////import org.firstinspires.ftc.teamcode.subsystems.Lift;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.TankDrive;
//import org.firstinspires.ftc.teamcode.subsystems.Vision;
//
//@TeleOp(name = "FGC TeleOp")
//public class FGCTeleOp extends CommandOpMode {
//    private TriggerReader triggerReader;
//    private SlewRateLimiter driverLimiter;
//    private SlewRateLimiter turnLimiter;
//
////    private List<LynxModule> allHubs;
//
//    private TankDrive tankDrive;
//    private Ascent ascent;
//    private Vision vision;
//    private Shooter shooter;
////    private Lift lift;
////    private Intake intake;
//    private GamepadEx gamepadEx1, gamepadEx2;
//    public State state;
//
//
//    public enum State{
//        FREE,
//        VISION,
//        SHOOT
//    }
//
//
//    @Override
//    public void initialize() {
//        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        CommandScheduler.getInstance().reset();
//
////        for(LynxModule hub : allHubs) {
////            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
////        }
//
//        //Subsystems Initialization
//        tankDrive = new TankDrive(hardwareMap);
//        ascent = new Ascent(hardwareMap);
////        vision = new Vision(telemetry, hardwareMap);
//        shooter = new Shooter(hardwareMap);
//
//        gamepadEx1 = new GamepadEx(gamepad1);
//
//        driverLimiter = new SlewRateLimiter(4);
////        turnLimiter = new SlewRateLimiter(3);
//
//        state = State.FREE;
//
//        tankDrive.setDefaultCommand(
//            new TankDriveCommand(
//                tankDrive,
//                () -> -driverLimiter.calculate(gamepadEx1.getLeftY()) ,
//                () -> gamepadEx1.getRightX(),
//                    () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5));
//
//        gamepadEx1
//                .getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new InstantCommand(()->ascent.ascentCloseloop()));
//
//        gamepadEx1
//                .getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new InstantCommand(()->ascent.reset()));
//
//        gamepadEx1
//                .getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(()-> ascent.ascentOpenloop()));
//
//        gamepadEx1
//                .getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(()-> ascent.ascentOpenloopDown()));
//
////        gamepadEx1
////                .getGamepadButton(GamepadKeys.Button.A)
////                .whenPressed(
////                        new ConditionalCommand(
////                                new InstantCommand(()->this.state=State.FREE),
////                                new InstantCommand(()->this.state=State.VISION),
////                                ()->this.state == State.VISION));
//
//        gamepadEx1
//                .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whileHeld(new InstantCommand(()-> vision.driveWithVision(tankDrive,telemetry,true)));
//
//        gamepadEx1
//                .getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new InstantCommand(()->shooter.StartShoot(telemetry)));
//        gamepadEx1
//                .getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new InstantCommand(()->shooter.StopShoot(telemetry)));
//    }
//
//    @Override
//    public void run() {
////        for(LynxModule hub : allHubs) {
////            hub.clearBulkCache();
////        }
//        CommandScheduler.getInstance().run();
//        updateTelemetry(telemetry);
//    }
//
//    public int getDPADAngle(GamepadEx gamepad) {
//        if(gamepad == null) return -1;
//        boolean[] buttonArray = {
//                gamepad.getButton(GamepadKeys.Button.DPAD_LEFT),
//                gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT),
//                gamepad.getButton(GamepadKeys.Button.DPAD_UP),
//                gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)
//        };
//        if(buttonArray[0] && !buttonArray[1] && !buttonArray[2] && !buttonArray[3]) {
//            return 180;
//        }
//        if(!buttonArray[0] && buttonArray[1] && buttonArray[2] && buttonArray[3]) {
//            return 0;
//        }
//        if(!buttonArray[0] && !buttonArray[1] && buttonArray[2] && !buttonArray[3]) {
//            return 90;
//        }
//        if(!buttonArray[0] && !buttonArray[1] && !buttonArray[2] && buttonArray[3]) {
//            return 270;
//        }
//        if(buttonArray[0] && !buttonArray[1] && buttonArray[2] && !buttonArray[3]) {
//            return 135;
//        }
//        if(buttonArray[0] && !buttonArray[1] && !buttonArray[2] && buttonArray[3]) {
//            return 225;
//        }
//        if(!buttonArray[0] && buttonArray[1] && buttonArray[2] && !buttonArray[3]) {
//            return 45;
//        }
//        if(buttonArray[0] && buttonArray[1] && !buttonArray[2] && buttonArray[3]) {
//            return 315;
//        }
//
//        return -1;
//    }
//
//
//
//}
