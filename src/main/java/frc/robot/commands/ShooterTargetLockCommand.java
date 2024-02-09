// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTargetLockCommand extends Command {
  /** Creates a new ShooterCommand. */
  ShooterSubsystem shooterSubsystem;

  DrivebaseSubsystem drivebaseSubsystem;

  public ShooterTargetLockCommand(ShooterSubsystem shooterSubsystem, DrivebaseSubsystem drivebaseSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.drivebaseSubsystem = drivebaseSubsystem;
    addRequirements(shooterSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.calculateWristTargetDegrees(
        drivebaseSubsystem.getPose(),
        drivebaseSubsystem.getChassisSpeeds().vxMetersPerSecond,
        drivebaseSubsystem.getChassisSpeeds().vyMetersPerSecond);
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
    return shooterSubsystem.isDone();
  }
}
