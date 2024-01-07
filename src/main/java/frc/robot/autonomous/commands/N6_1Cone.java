// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.EngageCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.IntakeModeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import frc.util.NodeSelectorUtility.Height;
import frc.util.NodeSelectorUtility.NodeType;

public class N6_1Cone extends SequentialCommandGroup {
  /** Creates a new N2MobilityEngage. */
  public N6_1Cone(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem) {

    addCommands(
        new SetZeroModeCommand(elevatorSubsystem)
            .deadlineWith(new IntakeModeCommand(intakeSubsystem, Modes.INTAKE, () -> true)),
        new ScoreCommand(
            intakeSubsystem,
            elevatorSubsystem,
            Constants.SCORE_STEP_MAP.get(NodeType.CONE.atHeight(Height.HIGH)),
            2),
        new ElevatorPositionCommand(elevatorSubsystem, () -> Elevator.Setpoints.STOWED)
            .deadlineWith(new IntakeModeCommand(intakeSubsystem, Modes.OFF)));
  }
}
