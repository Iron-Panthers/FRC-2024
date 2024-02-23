package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Lights.Colors;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.RGBSubsystem.MessagePriority;
import frc.robot.subsystems.RGBSubsystem.PatternTypes;
import frc.robot.subsystems.ShooterSubsystem;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class AdvancedIntakeCommand extends SequentialCommandGroup {

  public AdvancedIntakeCommand(
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      RGBSubsystem rgbSubsystem) {
    addCommands(
        new IntakeCommand(intakeSubsystem, shooterSubsystem)
            .alongWith(
                new RGBCommand(
                    rgbSubsystem,
                    Colors.RED,
                    PatternTypes.BOUNCE,
                    MessagePriority.C_INTAKE_STATE_CHANGE)),
        new RGBCommand(
            rgbSubsystem,
            Colors.ORANGE,
            PatternTypes.BOUNCE,
            MessagePriority.C_INTAKE_STATE_CHANGE),
        new ParallelCommandGroup(
            new UnstuckIntakeCommand(intakeSubsystem)
                .withTimeout(3)
                .andThen(new StopIntakeCommand(intakeSubsystem)),
            new PivotAngleCommand(shooterSubsystem, 30)
                .andThen(new ShooterRampUpCommand(shooterSubsystem))));
  }
}
