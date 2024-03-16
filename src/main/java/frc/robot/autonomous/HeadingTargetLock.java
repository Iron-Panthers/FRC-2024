package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Pivot;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem.AutoRotationOverride;

public class HeadingTargetLock extends Command {
  private DrivebaseSubsystem drivebaseSubsystem;

  private final Pose2d targetPoint;

  private double targetAngle;

  public HeadingTargetLock(DrivebaseSubsystem drivebaseSubsystem) {
    this.drivebaseSubsystem = drivebaseSubsystem;

    var alliance = DriverStation.getAlliance();
    this.targetPoint =
        (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            ? Pivot.RED_SPEAKER_POSE
            : Pivot.BLUE_SPEAKER_POSE;

    targetAngle = 0;

    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void execute() {
    targetAngle =
        (double)
                -Math.toDegrees(
                    Math.atan2(
                        (targetPoint.getY() - drivebaseSubsystem.getPose().getY()),
                        (targetPoint.getX() - drivebaseSubsystem.getPose().getX())))
            + (DriverStation.getAlliance().get() == Alliance.Blue ? 180 : 0);

    drivebaseSubsystem.driveAutonomousAngle(targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.setAutoRotOverride(AutoRotationOverride.PATHPLANNER);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
