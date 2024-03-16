package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class SpeakerLockCommand extends ParallelCommandGroup {
  public SpeakerLockCommand(DrivebaseSubsystem drivebaseSubsystem, PivotSubsystem pivotSubsystem) {
    addCommands(
        new TargetLockCommand(drivebaseSubsystem, () -> 0, () -> 0),
        new PivotTargetLockCommand(pivotSubsystem, drivebaseSubsystem));
  }
}
