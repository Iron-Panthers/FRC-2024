// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This command takes a drive stick and nothing else, and drives the robot. It does not send any
 * rotation. This allows the command scheduler to negotiate the different types of rotation without
 * us worrying about it - if a new rotation is introduced, it takes priority, without rotation
 * inside default command always getting lowest priority.
 */
public class DefaultDriveCommand extends Command {
  private final DrivebaseSubsystem drivebaseSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  private final BooleanSupplier isRobotRelativeForwardSupplier;
  // private final BooleanSupplier isRobotRelativeBackwardSupplier;

  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      BooleanSupplier isRobotRelativeForwardSupplier) {
    // BooleanSupplier isRobotRelativeBackwardSupplier) {

    this.drivebaseSubsystem = drivebaseSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.isRobotRelativeForwardSupplier = isRobotRelativeForwardSupplier;
    // this.isRobotRelativeBackwardSupplier = isRobotRelativeBackwardSupplier;

    addRequirements(drivebaseSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = translationXSupplier.getAsDouble();
    double y = translationYSupplier.getAsDouble();
    Boolean forwardRelative = isRobotRelativeForwardSupplier.getAsBoolean();
    // Boolean backwardRelative = isRobotRelativeBackwardSupplier.getAsBoolean();

    drivebaseSubsystem.drive(
        DrivebaseSubsystem.produceChassisSpeeds(
            forwardRelative,
            // backwardRelative,
            x,
            y,
            0,
            drivebaseSubsystem.getDriverGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
