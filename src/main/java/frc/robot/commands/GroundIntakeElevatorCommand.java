package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Modes;
import java.util.function.BooleanSupplier;

public class GroundIntakeElevatorCommand extends Command {

  private ElevatorSubsystem elevatorSubsystem;
  private BooleanSupplier isCone;

  public GroundIntakeElevatorCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier isCone) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.isCone = isCone;

    addRequirements(this.elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setMode(Modes.POSITION_CONTROL);
  }

  @Override
  public void execute() {
    if (isCone.getAsBoolean()) {
      elevatorSubsystem.setTargetState(Constants.Elevator.Setpoints.GROUND_INTAKE_CONE);
    } else {
      elevatorSubsystem.setTargetState(Constants.Elevator.Setpoints.GROUND_INTAKE_CUBE);
    }
    elevatorSubsystem.setMode(Modes.POSITION_CONTROL);
  }
}
