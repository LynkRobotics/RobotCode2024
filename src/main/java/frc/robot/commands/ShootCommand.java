// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LEDSubsystem.TempState;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final IndexSubsystem index;
    private boolean feeding = false;
    DoubleSupplier topSupplier = null;
    DoubleSupplier bottomSupplier = null;
    private boolean cancelled = false;
    private boolean gone = false;
    private Timer postShotTimer = new Timer();

    public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index) {
        addRequirements(shooter, index);
        this.shooter = shooter;
        this.index = index;
    }

    public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index, DoubleSupplier topSupplier,
            DoubleSupplier bottomSupplier) {
        this(shooter, index);
        this.topSupplier = topSupplier;
        this.bottomSupplier = bottomSupplier;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LEDSubsystem.setTempState(TempState.SHOOTING);
        cancelled = false;
        feeding = false;
        gone = false;

        if (topSupplier != null && bottomSupplier != null) {
            shooter.shoot(topSupplier.getAsDouble(), bottomSupplier.getAsDouble());
        } else {
            if (!index.haveNote()) {
                DogLog.log("Shooter/Status", "ERROR: Cancelling ShootCommand without note");
                cancelled = true;
            }
            if (!shooter.shoot()) {
                DogLog.log("Shooter/Status", "ERROR: Cancelling ShootCommand due to shoot() failure");
                cancelled = true;
            }
            if (cancelled) {
                LEDSubsystem.setTempState(TempState.ERROR);
                cancel();
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (cancelled) {
            return;
        }
        if (!feeding && shooter.isReady(false)) {
            index.feed();
            feeding = true;
        }
        if (!gone && !index.haveNote()) {
            postShotTimer.restart();
            gone = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop feeding
        index.stop();

        // Restore idle speed
        shooter.idle();

        // Restore default shot
        shooter.setNextShot(null);

        // Adjust LED state
        if (interrupted) {
            LEDSubsystem.setTempState(TempState.ERROR);
        } else {
            LEDSubsystem.clearTempState();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (cancelled) {
            return true;
        }
        if (gone && postShotTimer.hasElapsed(Constants.Shooter.postShotTimeout)) {
            return true;
        }
        return false;
    }
}