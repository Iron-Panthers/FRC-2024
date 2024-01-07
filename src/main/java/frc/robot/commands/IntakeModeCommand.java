// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import java.util.function.BooleanSupplier;

public class IntakeModeCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private Modes mode;
  private BooleanSupplier isCone;
  private boolean isGroundIntake;

  /** Creates a new IntakeCommand. */
  public IntakeModeCommand(IntakeSubsystem intakeSubsystem, Modes mode, BooleanSupplier isCone, boolean isGroundIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.mode = mode;
    this.isCone = isCone;
    this.isGroundIntake = isGroundIntake;
    addRequirements(intakeSubsystem);
  }

  public IntakeModeCommand(IntakeSubsystem intakeSubsystem, Modes mode, BooleanSupplier isCone) {
    this(intakeSubsystem, mode, isCone, false);
  }

  public IntakeModeCommand(IntakeSubsystem intakeSubsystem, Modes mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.mode = mode;
    this.isCone = null;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    intakeSubsystem.setMode(mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isCone != null) {
      intakeSubsystem.setIsCone(isCone.getAsBoolean());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!isGroundIntake) {
      if (intakeSubsystem.getMode() == Modes.HOLD || intakeSubsystem.getMode() == Modes.OFF) {
        return true;
      }
    } else {
      if(intakeSubsystem.getFilterOutput() > Intake.GROUND_CUBE_STATOR_LIMIT) {
        return true;
      }
    }
    return false;
  }
}
