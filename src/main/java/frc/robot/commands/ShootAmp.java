// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class ShootAmp extends Command {
  private ShooterSubsystem shooterSubsystem;

  /** Creates a new ShootAmp. */
  public ShootAmp(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (shooterSubsystem.getMode().equals(ShooterMode.RAMP_AMP_BACK)) {
      shooterSubsystem.setShooterMode(ShooterMode.SHOOT_AMP_BACK);
    } else {
      shooterSubsystem.setShooterMode(ShooterMode.SHOTO_AMP_FORWARD);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !shooterSubsystem.isBeamBreakSensorTriggered();
  }
}
