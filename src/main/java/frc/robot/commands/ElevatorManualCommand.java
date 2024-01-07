// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Modes;
import java.util.function.DoubleSupplier;

public class ElevatorManualCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  DoubleSupplier extensionRate;
  DoubleSupplier angleRate;

  /** Creates a new AngleArmCommand. */
  public ElevatorManualCommand(
      ElevatorSubsystem elevatorSubsystem, DoubleSupplier extentionRate, DoubleSupplier angleRate) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.extensionRate = extentionRate;
    this.angleRate = angleRate;

    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setMode(Modes.PERCENT_CONTROL);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // elevatorSubsystem.setTargetExtensionInches(
    //     MathUtil.clamp(
    //         (elevatorSubsystem.getTargetExtension() + extensionRate.getAsDouble()),
    //         Elevator.MIN_EXTENSION_INCHES,
    //         Elevator.MAX_EXTENSION_INCHES));
    // elevatorSubsystem.setTargetAngle(
    //     MathUtil.clamp(
    //         (elevatorSubsystem.getCurrentAngleDegrees() + angleRate.getAsDouble()),
    //         Elevator.MIN_ANGLE_DEGREES,
    //         Elevator.MAX_ANGLE_DEGREES));
    // elevatorSubsystem.setTargetExtensionInches(
    //     MathUtil.clamp(
    //         (elevatorSubsystem.getExtensionInches() + extensionRate.getAsDouble()),
    //         Elevator.MIN_EXTENSION_INCHES,
    //         Elevator.MAX_EXTENSION_INCHES));
    elevatorSubsystem.setPercentControl(extensionRate.getAsDouble(), angleRate.getAsDouble());
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
