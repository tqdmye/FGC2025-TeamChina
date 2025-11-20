package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.RunCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.gamepad.TriggerReader;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
//import org.firstinspires.ftc.teamcode.commands.TankDriveCommand;
//import org.firstinspires.ftc.teamcode.common.constants.Constants;
//import org.firstinspires.ftc.teamcode.common.util.SlewRateLimiter;
//import org.firstinspires.ftc.teamcode.subsystems.Ascent;
//import org.firstinspires.ftc.teamcode.subsystems.Holder;
//import org.firstinspires.ftc.teamcode.subsystems.Pusher;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.TankDrive;
//import org.firstinspires.ftc.teamcode.subsystems.Vision;
//
//@TeleOp(name = "FGC TeleOp Vision")
//public class FGCTeleOpVision extends CommandOpMode {
//  private TriggerReader triggerReader;
//  private SlewRateLimiter driverLimiter;
//  private SlewRateLimiter turnLimiter;
//
//  //    private List<LynxModule> allHubs;
//
//  private TankDrive tankDrive;
//  private Ascent ascent;
//  private Vision vision;
//  private Shooter shooter;
//  private Pusher pusher;
//  private Holder holder;
//  private GamepadEx gamepadEx1, gamepadEx2;
//  public State state;
//  public int frontCount=0;
//  public int backCount=0;
//
//  public enum State {
//    FREE,
//    VISION,
//    SHOOT
//  }
//
//  @Override
//  public void initialize() {
//    this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//    CommandScheduler.getInstance().reset();
//
//    //        for(LynxModule hub : allHubs) {
//    //            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//    //        }
//
//    // Subsystems Initialization
//    tankDrive = new TankDrive(hardwareMap);
//    ascent = new Ascent(hardwareMap);
//    vision = new Vision(telemetry, hardwareMap);
//    shooter = new Shooter(telemetry, hardwareMap);
//    pusher = new Pusher(telemetry, hardwareMap);
//    holder = new Holder(telemetry, hardwareMap);
//
//    gamepadEx1 = new GamepadEx(gamepad1);
//    gamepadEx2 = new GamepadEx(gamepad2);
//
//    driverLimiter = new SlewRateLimiter(4);
//    turnLimiter = new SlewRateLimiter(3);
//
//    state = State.FREE;
//
//    tankDrive.setDefaultCommand(
//        new TankDriveCommand(
//            tankDrive,
//            () -> -driverLimiter.calculate(gamepadEx1.getLeftY()),
//            () -> gamepadEx1.getRightX(),
//            () -> gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5));
//
//            gamepadEx1
//                    .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                    .whileHeld(new RunCommand(() -> vision.driveWithVision(tankDrive, telemetry, true)));
//
//    gamepadEx1
//        .getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//        .whenPressed(new InstantCommand(() -> pusher.push()));
//
//    gamepadEx1
//        .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//        .whenPressed(
//            new ParallelCommandGroup(
//                new InstantCommand(() -> pusher.accelerate()),
//                new InstantCommand(() -> holder.open())));
//
//    gamepadEx1
//            .getGamepadButton(GamepadKeys.Button.DPAD_UP)
//            .whenPressed(new InstantCommand(()->++frontCount));
//
//    gamepadEx1
//            .getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//            .whenPressed(new InstantCommand(()->--frontCount));
//
//    gamepadEx1
//            .getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//            .whenPressed(new InstantCommand(()->++backCount));
//
//    gamepadEx1
//            .getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//            .whenPressed(new InstantCommand(()->--backCount));
//
//    gamepadEx2.getGamepadButton(GamepadKeys.Button.Y).whileHeld(new ShooterCommand(shooter));
//    gamepadEx2
//        .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//        .whileHeld(
//            new ParallelCommandGroup(
//                new InstantCommand(() -> pusher.shoot()),
//                new InstantCommand(
//                    () ->
//                        shooter.setShooterVelocity(
//                            Constants.FRONT_SHOOTER_VELOCITY.value+frontCount*20,
//                            Constants.BACK_SHOOTER_VELOCITY.value+backCount*20))))
//        .whenReleased(new InstantCommand(() -> shooter.setShooterVelocity(0, 0)));
//    gamepadEx2
//        .getGamepadButton(GamepadKeys.Button.X)
//        .whileHeld(new InstantCommand(() -> shooter.intake()))
//        .whenReleased(new InstantCommand(() -> shooter.reset()));
//
//    gamepadEx2
//        .getGamepadButton(GamepadKeys.Button.B)
//        .whenPressed(new InstantCommand(() -> shooter.counterIntake()))
//        .whenReleased(new InstantCommand(() -> shooter.reset()));
//
//    gamepadEx2
//        .getGamepadButton(GamepadKeys.Button.A)
//        .whenPressed(new InstantCommand(() -> shooter.counterPreShooterAndBlender()))
//        .whenReleased(new InstantCommand(() -> shooter.reset()));
//
//    gamepadEx2
//        .getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//        .whenPressed(new InstantCommand(() -> holder.prepare()));
//
//    gamepadEx2
//        .getGamepadButton(GamepadKeys.Button.DPAD_UP)
//        .whenPressed(new InstantCommand(() -> holder.hold()));
//
//    gamepadEx2
//        .getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//        .whenPressed(new InstantCommand(() -> holder.open()));
//
//    //        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//    //                .whileHeld(
//    //                        new ParallelCommandGroup(
//    //                                new InstantCommand(()->pusher.accelerate()),
//    //                                new
//    // InstantCommand(()->shooter.setShooterVelocity(Constants.FRONT_SHOOTER_VELOCITY.value,0))))
//    //                .whenReleased(new InstantCommand(()->shooter.setShooterVelocity(0,0)));
//    gamepadEx2
//        .getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//        .whileHeld(new InstantCommand(() -> shooter.stopIntake()))
//        .whenReleased(new InstantCommand(() -> shooter.state = Shooter.ShooterState.INTAKE));
//
//    gamepadEx2
//        .getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//        .whenPressed(new InstantCommand(() -> ascent.brakeAscent()));
//  }
//
//  private boolean lastRightTriggerPressed = false;
//
//  @Override
//  public void run() {
//    pusher.manualControl(-gamepadEx2.getRightY() * 0.5);
//
//    ascent.ascentUpDual(gamepadEx2.getLeftY());
//    if (gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
//      ascent.unlockAscentDual();
//    }
//
//    boolean rightTriggerPressed = gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
//
//    // 按住时 shooter 转动
//    if (rightTriggerPressed) {
//      pusher.accelerate();
//      shooter.setShooterVelocity(Constants.FRONT_SHOOTER_VELOCITY.value, 0);
//    }
//
//    // 松开瞬间时停止 shooter
//    if (lastRightTriggerPressed && !rightTriggerPressed) {
//      shooter.setShooterVelocity(0, 0);
//    }
//
//    lastRightTriggerPressed = rightTriggerPressed;
//
//    //        if(gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.5){
//    //            if(shooter.state != Shooter.ShooterState.ACCELERATING){
//    //                pusher.accelerate();
//    //                shooter.setShooterVelocity(Constants.FRONT_SHOOTER_VELOCITY.value,0);
//    //                shooter.state = Shooter.ShooterState.ACCELERATING;
//    //            }
//    //            else if(shooter.state == Shooter.ShooterState.ACCELERATING){
//    //                shooter.setShooterVelocity(0,0);
//    //                shooter.state = Shooter.ShooterState.FREE;
//    //            }
//    //        }
//
//    CommandScheduler.getInstance().run();
//    telemetry.addData("Current Angle", pusher.currentAngle);
//    telemetry.addData("Target Angle", pusher.targetAngle);
//    telemetry.addData("State", pusher.currentState);
//    telemetry.addData("front shooter target velocity", Constants.FRONT_SHOOTER_VELOCITY.value+frontCount*20);
//    telemetry.addData("front shooter velocity", shooter.frontShooter.getVelocity());
//    telemetry.addData("back shooter target velocity", Constants.BACK_SHOOTER_VELOCITY.value+backCount*20);
//    telemetry.addData("back shooter velocity", shooter.backShooter.getVelocity());
//    updateTelemetry(telemetry);
//  }
//}
