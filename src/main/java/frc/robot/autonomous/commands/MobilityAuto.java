// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPlaceCommand;
import frc.robot.commands.IntakeModeCommand;
import frc.robot.commands.SetZeroModeCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import frc.robot.subsystems.RGBSubsystem;
import frc.util.pathing.AlliancePose2d;
import frc.util.pathing.RubenManueverGenerator;
import java.util.Optional;

public class MobilityAuto extends SequentialCommandGroup {
  /** Creates a new MobilityAuto. */
  public MobilityAuto(
      RubenManueverGenerator manueverGenerator,
      DrivebaseSubsystem drivebaseSubsystem,
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      RGBSubsystem rgbSubsystem,
      AlliancePose2d finalPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new DriveToPlaceCommand(
                drivebaseSubsystem,
                manueverGenerator,
                finalPose::get,
                () -> 0,
                () -> 0,
                () -> false,
                Optional.of(rgbSubsystem),
                Optional.empty())
            .alongWith(new SetZeroModeCommand(elevatorSubsystem))
            .alongWith(new IntakeModeCommand(intakeSubsystem, Modes.HOLD))
            .andThen(new InstantCommand(drivebaseSubsystem::zeroGyroscope, drivebaseSubsystem)));
  }
}
