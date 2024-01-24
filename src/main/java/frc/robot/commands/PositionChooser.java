// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.Modes;

public class PositionChooser extends CommandBase {
  /** Creates a new PositionChooser. */
  ClimberSubsystem climberSubsystem;

  double targetExtension;

  public PositionChooser(ClimberSubsystem climberSubsystem, double targetExtension) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberSubsystem = climberSubsystem;
    this.targetExtension = targetExtension;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    climberSubsystem.setMode(Modes.POSITION_CONTROL);
    climberSubsystem.setTargetExtension(targetExtension);
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
