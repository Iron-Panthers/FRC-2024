// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.Modes;
import java.util.function.DoubleSupplier;

public class ClimberManualCommand extends Command {

  private ClimberSubsystem subsystem;
  private DoubleSupplier extensionRate;

  /** Creates a new ClimberCommand. */
  public ClimberManualCommand(ClimberSubsystem subsystem, DoubleSupplier extensionRate) {
    this.subsystem = subsystem;
    this.extensionRate = extensionRate;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setMode(Modes.PERCENT_CONTROL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.setPercentPower(extensionRate.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
