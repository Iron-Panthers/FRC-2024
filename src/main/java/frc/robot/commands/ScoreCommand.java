// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCommand extends SequentialCommandGroup {
  public static record ScoreStep(
      Optional<ElevatorState> elevatorState,
      Optional<IntakeSubsystem.Modes> intakeState,
      boolean isCube,
      boolean isPausePoint) {
    public ScoreStep(
        ElevatorState elevatorState, IntakeSubsystem.Modes intakeState, boolean isCube) {
      this(Optional.of(elevatorState), Optional.of(intakeState), isCube, false);
    }

    public ScoreStep(ElevatorState elevatorState) {
      this(Optional.of(elevatorState), Optional.empty(), false, false);
    }

    public ScoreStep(IntakeSubsystem.Modes intakeState, boolean isCube) {
      this(Optional.empty(), Optional.of(intakeState), isCube, false);
    }

    public ScoreStep canWaitHere() {
      return new ScoreStep(elevatorState, intakeState, false, true);
    }
  }

  private static class AwaitTriggerPressed extends Command {
    private final Trigger trigger;
    private boolean hasBeenFalse;

    public AwaitTriggerPressed(Trigger trigger) {
      this.trigger = trigger;
    }

    @Override
    public void initialize() {
      hasBeenFalse = false;
    }

    @Override
    public void execute() {
      if (!trigger.getAsBoolean()) {
        hasBeenFalse = true;
      }
    }

    @Override
    public boolean isFinished() {
      return hasBeenFalse && trigger.getAsBoolean();
    }
  }

  private Command createStep(ScoreStep scoreStep) {
    var elevatorState = scoreStep.elevatorState();
    var intakeState = scoreStep.intakeState();
    var isCube = scoreStep.isCube();
    if (elevatorState.isPresent() && intakeState.isPresent()) {
      return new ElevatorPositionCommand(elevatorSubsystem, () -> elevatorState.get())
          .deadlineWith(new IntakeModeCommand(intakeSubsystem, intakeState.get(), () -> isCube));
    } else if (elevatorState.isPresent()) {
      return new ElevatorPositionCommand(elevatorSubsystem, () -> elevatorState.get());
    } else if (intakeState.isPresent()) {
      return new IntakeModeCommand(intakeSubsystem, intakeState.get(), () -> isCube);
    } else {
      throw new IllegalArgumentException("ScoreStep must have at least one state");
    }
  }

  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public ScoreCommand(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      List<ScoreStep> scoreSteps,
      Trigger trigger) {
    this(intakeSubsystem, elevatorSubsystem, scoreSteps, Optional.of(trigger), Optional.empty());
  }

  public ScoreCommand(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      List<ScoreStep> scoreSteps) {
    this(intakeSubsystem, elevatorSubsystem, scoreSteps, Optional.empty(), Optional.empty());
  }

  public ScoreCommand(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      List<ScoreStep> scoreSteps,
      double stepDeadline) {
    this(
        intakeSubsystem,
        elevatorSubsystem,
        scoreSteps,
        Optional.empty(),
        Optional.of(stepDeadline));
  }

  /** Creates a new ScoreCommand. */
  private ScoreCommand(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      List<ScoreStep> scoreSteps,
      Optional<Trigger> trigger,
      Optional<Double> stepDeadline) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;

    for (ScoreStep scoreStep : scoreSteps) {
      if (trigger.isEmpty())
        addCommands(
            stepDeadline.isPresent()
                ? createStep(scoreStep).withTimeout(stepDeadline.get())
                : createStep(scoreStep));
      else
        addCommands(
            scoreStep.isPausePoint()
                ? (new AwaitTriggerPressed(trigger.get())).deadlineWith(createStep(scoreStep))
                : (new AwaitTriggerPressed(trigger.get())).raceWith(createStep(scoreStep)));
    }
  }

  public static List<ScoreCommand> splitAlongPausePoints(
      IntakeSubsystem intakeSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      List<ScoreStep> scoreSteps,
      double stepDeadline) {
    var scoreCommands = new ArrayList<ScoreCommand>();

    int start = 0;
    int end = 0;
    while (end < scoreSteps.size()) {
      if (scoreSteps.get(end).isPausePoint()) {
        // System.out.printf("start: %d end: %d%n", start, end);
        // System.out.println(scoreSteps.subList(start, end + 1));
        scoreCommands.add(
            new ScoreCommand(
                intakeSubsystem,
                elevatorSubsystem,
                scoreSteps.subList(start, end + 1),
                stepDeadline));
        start = end + 1;
      }
      end++;
    }

    // System.out.printf("start: %d end: %d%n", start, end);
    // System.out.println(scoreSteps.subList(start, end));
    scoreCommands.add(
        new ScoreCommand(
            intakeSubsystem, elevatorSubsystem, scoreSteps.subList(start, end), stepDeadline));

    return scoreCommands;
  }
}
