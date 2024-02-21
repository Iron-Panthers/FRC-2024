package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class PivotManualCommand extends Command {

  ShooterSubsystem shooterSubsystem;

  DoubleSupplier voltage;

  public PivotManualCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier voltage) {
    this.shooterSubsystem = shooterSubsystem;
    this.voltage = voltage;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void execute() {
    shooterSubsystem.setPivotVoltage(voltage.getAsDouble() * Shooter.PIVOT_MAX_VOLTAGE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
