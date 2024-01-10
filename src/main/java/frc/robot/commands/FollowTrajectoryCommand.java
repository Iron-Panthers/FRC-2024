// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;
import java.util.function.Supplier;

/** Command to interface with the advanced trajectory follower, with automatic path mirroring. */
public class FollowTrajectoryCommand extends Command {
  private final DrivebaseSubsystem drivebaseSubsystem;
  private final Supplier<PathPlannerTrajectory> trajectory;
  /**
   * True if the swerve drive subsystem should localize to the trajectory's starting point in the
   * initialization block. Calls the underlying SwerveDriveOdometry.resetOdometry(pose, angle).
   */
  private final boolean localizeToStartPose;

  /**
   * Creates a new FollowTrajectoryCommand. If you would like to localize to the start pose of the
   * trajectory, instead use the constructor with a boolean parameter.
   *
   * @param trajectory The desired trajectory to track.
   * @param drivebaseSubsystem The instance of the Drivebase subsystem (should come from
   *     RobotContainer)
   */
  public FollowTrajectoryCommand(
      Supplier<PathPlannerTrajectory> trajectory, DrivebaseSubsystem drivebaseSubsystem) {
    this.trajectory = trajectory;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.localizeToStartPose = false;
    addRequirements(drivebaseSubsystem);
  }

  /**
   * Creates a new FollowTrajectoryCommand. Adds a parameter to optionally localize to the start
   * point of this trajectory.
   *
   * @param trajectory The desired trajectory to track.
   * @param localizeToStartPose If true, the drivebase will reset odometry to trajectory_state[0]
   * @param drivebaseSubsystem The instance of the Drivebase subsystem (should come from
   *     RobotContainer)
   */
  public FollowTrajectoryCommand(
      Supplier<PathPlannerTrajectory> trajectory,
      boolean localizeToStartPose,
      DrivebaseSubsystem drivebaseSubsystem) {
    this.trajectory = trajectory;
    this.drivebaseSubsystem = drivebaseSubsystem;
    this.localizeToStartPose = localizeToStartPose;
    addRequirements(drivebaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
