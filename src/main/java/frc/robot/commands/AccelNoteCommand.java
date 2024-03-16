package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class AccelNoteCommand extends Command {
  private ShooterSubsystem shooterSubsystem;

  public AccelNoteCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.setShooterMode(ShooterMode.ACCEL_SECURE);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterMode(ShooterMode.RAMP_SPEAKER);
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.isReadyToShoot();
  }
}
