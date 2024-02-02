// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

  IntakeSubsystem intakeSubsystem;

  IntakeSubsystem.Modes intakeMode;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeSubsystem.Modes intakeMode) {
    this.intakeSubsystem = intakeSubsystem;
    this.intakeMode = intakeMode;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setIntakeMode(intakeMode);
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
    // if(intakeMode == IntakeSubsystem.Modes.INTAKE){
    //     return intakeSubsystem.getSensorOutput();
    // }else if(intakeMode == IntakeSubsystem.Modes.OUTTAKE){
    //     return !intakeSubsystem.getSensorOutput();
    // }
    return true;
  }
}
