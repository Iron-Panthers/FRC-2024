// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundPickupCommand extends SequentialCommandGroup {
  /** Creates a new GroundPickupCommand. */
  public GroundPickupCommand(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      BooleanSupplier isCone) {
    // Add your commands in the addCommands() call, e.g.

    addCommands(
        new IntakeModeCommand(intakeSubsystem, Modes.INTAKE, isCone, true)
            .deadlineWith(new GroundIntakeElevatorCommand(elevatorSubsystem, isCone))
            .andThen(
                new ElevatorPositionCommand(elevatorSubsystem, () -> Elevator.Setpoints.STOWED)));
  }
}
