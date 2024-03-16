package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterMode;

public class MaintainShooterCommand extends Command {
  ShooterSubsystem shooterSubsystem;

  public MaintainShooterCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    shooterSubsystem.setShooterMode(ShooterMode.MAINTAIN_VELOCITY);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
