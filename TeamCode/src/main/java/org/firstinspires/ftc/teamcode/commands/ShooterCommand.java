package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterCommand extends CommandBase {
    private final Shooter shooter;

    public ShooterCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
//        shooter.setShooterVelocity(Constants.FRONT_SHOOTER_VELOCITY.value, Constants.BACK_SHOOTER_VELOCITY.value);
    }

    @Override
    public void execute() {
        if (shooter.getFrontShooterVelocity() >= Constants.SHOOTER_MIN_VELOCITY.value) {
            shooter.shooting(1.0);
        } else {
            shooter.shooting(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.reset();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

