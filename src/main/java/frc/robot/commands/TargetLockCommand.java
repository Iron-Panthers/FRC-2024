// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.util.Util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.*;

/**
 * This command takes a drive angle and a target x,y coordinate, and snaps the robot to face the
 * target. This is useful to lock the robot to fixed target with a button.
 */
public class TargetLockCommand extends Command {
  private final DrivebaseSubsystem drivebaseSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  private final Translation2d targetPoint;

  private int targetAngle;
  private Supplier<Pose2d> pose;

  /** Creates a new TargetLockCommand. */
  public TargetLockCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Translation2d targetPoint) {

    this.drivebaseSubsystem = drivebaseSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.targetPoint = targetPoint;

    this.pose = () -> this.drivebaseSubsystem.getPose();

    targetAngle = 0;

    addRequirements(drivebaseSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetAngle =
        (int) // may want to change to double for more accuracy (likely unnecessary)
            - Math.toDegrees(Math.atan2(
                (targetPoint.getY() - pose.get().getY()),
                    ( targetPoint.getX() - pose.get().getX())
                  ));
    double x = translationXSupplier.getAsDouble();
    double y = translationYSupplier.getAsDouble();

    drivebaseSubsystem.driveAngle(
        new Pair<Double, Double>(x, y), targetAngle // the desired angle, gyro relative
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return Util.epsilonZero(
    //         Util.relativeAngularDifference(
    //             drivebaseSubsystem.getDriverGyroscopeRotation(), targetAngle),
    //         Drive.ANGULAR_ERROR)
    //     && Util.epsilonEquals(drivebaseSubsystem.getRotVelocity(), 0, 10);
  }
}
