package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem.AutoRotationOverride;

public class HeadingAngle extends Command {
  private DrivebaseSubsystem drivebaseSubsystem;

  private double targetAngle;

  public HeadingAngle(DrivebaseSubsystem drivebaseSubsystem, double targetAngle) {
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.targetAngle = targetAngle;
  }

  @Override
  public void initialize() {
    drivebaseSubsystem.driveAutonomousAngle(targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.setAutoRotOverride(AutoRotationOverride.PATHPLANNER);
  }

  @Override
  public boolean isFinished() {
    // use in race condition w/ path/timer
    return false;
  }
}
