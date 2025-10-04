package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterCommand extends CommandBase {
    private final Shooter shooter;
    private final double targetVelocity;
    private final double minVelocity;

    public ShooterCommand(Shooter shooter, double targetVelocity, double minVelocity) {
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;
        this.minVelocity = minVelocity;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterVelocity(targetVelocity);
    }

    @Override
    public void execute() {
        if (shooter.getShooterVelocity() >= minVelocity) {
            shooter.shooting(1.0);
        } else {
            shooter.shooting(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

