// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem.Modes;
import frc.util.Util;
import java.util.function.Supplier;

public class ElevatorPositionCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private double targetExtension;
  private double targetAngle;
  private Supplier<ElevatorState> elevatorStateSupplier;

  /** Creates a new ElevatorPositionCommand. */
  // public ElevatorPositionCommand(
  //     ElevatorSubsystem subsystem, double targetExtension, double targetAngle) {
  //   this.elevatorSubsystem = subsystem;
  //   this.targetExtension = targetExtension;
  //   this.targetAngle = targetAngle;
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   addRequirements(elevatorSubsystem);
  // }

  // public ElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, ElevatorState
  // elevatorState) {
  //   // height and angle
  //   this.elevatorSubsystem = elevatorSubsystem;
  //   targetExtension = elevatorState.extension();
  //   targetAngle = elevatorState.angle();
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   addRequirements(elevatorSubsystem);
  // }

  public ElevatorPositionCommand(
      ElevatorSubsystem elevatorSubsystem, Supplier<ElevatorState> elevatorStateSupplier) {
    // height and angle
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorStateSupplier = elevatorStateSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setTargetExtensionInches(elevatorStateSupplier.get().extension());
    elevatorSubsystem.setTargetAngle(elevatorStateSupplier.get().angle());
    elevatorSubsystem.setMode(Modes.POSITION_CONTROL);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Util.epsilonEquals(
            elevatorSubsystem.getCurrentAngleDegrees(),
            elevatorSubsystem.getTargetAngle(),
            Elevator.ANGLE_EPSILON)
        && Util.epsilonEquals(
            elevatorSubsystem.getCurrentExtensionInches(),
            elevatorSubsystem.getTargetExtension(),
            Elevator.EXTENSION_EPSILON);
    // && extensionController.atSetpoint()
    // && angleController.atSetpoint();
    // add a condition to end the command when the elevator
    // reaches target position
    // look at 2022 and 2023 code
  }
}
