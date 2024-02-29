package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import java.util.function.DoubleSupplier;

public class PivotManualCommand extends Command {

  PivotSubsystem pivotSubsystem;

  DoubleSupplier joystickRate;

  public PivotManualCommand(PivotSubsystem pivotSubsystem, DoubleSupplier joystickRate) {
    this.pivotSubsystem = pivotSubsystem;
    this.joystickRate = joystickRate;

    addRequirements(pivotSubsystem);
  }

  @Override
  public void execute() {
    pivotSubsystem.setTargetDegrees(pivotSubsystem.getTargetDegrees() + joystickRate.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
