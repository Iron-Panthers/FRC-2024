package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class PivotManualCommand extends Command {

  ShooterSubsystem shooterSubsystem;

  DoubleSupplier targetRate;

  public PivotManualCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier targetRate) {
    this.shooterSubsystem = shooterSubsystem;
    this.targetRate = targetRate;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    double currentAngle = shooterSubsystem.getCurrentAngle();
    shooterSubsystem.setTargetDegrees(currentAngle + targetRate.getAsDouble());
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }

}
