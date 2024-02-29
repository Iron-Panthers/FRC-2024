// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotAngleCommand extends Command {
  /** Creates a new StoreShooterCommand. */
  PivotSubsystem pivotSubsystem;

  double angle;

  public PivotAngleCommand(PivotSubsystem pivotSubsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.pivotSubsystem = pivotSubsystem;
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotSubsystem.setTargetDegrees(angle);
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
    return pivotSubsystem.isAtTargetDegrees();
  }
}
