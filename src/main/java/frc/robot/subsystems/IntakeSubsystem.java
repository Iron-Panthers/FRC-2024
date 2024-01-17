// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Config;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  private final DigitalInput noteSensor;

  private Modes intakeMode;

  public enum Modes {
    Intake,
    Outtake,
    Hold,
    Idle
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);

    intakeMotor.setNeutralMode(NeutralModeValue.Coast);

    intakeMotor.clearStickyFaults();

    // Mode to tell the motor what speed to go at
    intakeMode = Modes.Hold; // default to hold

    // Beam break code - once the beam is broken we need to hold the piece in place.
    noteSensor = new DigitalInput(Constants.Intake.INTAKE_SENSOR_PORT);

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      tab.add("intake power", intakeMotor.getMotorVoltage().asSupplier());
    }
  }

  @Override
  public void periodic() {

    // check for a beam break
    if (noteSensor.get()) { // if the beam is broken
      intakeMode = Modes.Hold; // set the mode to hold to hold the note
    }

    SetIntakeMotorSpeed();

    SetIntakeMotorSpeed();
  }

  public void SetIntakeMotorSpeed() { // using the current mode, set the motor speed
    switch (intakeMode) {
      case Intake:
        intakeMotor.set(Constants.Intake.INTAKE_MOTOR_SPEED);
        break;
      case Outtake:
        intakeMotor.set(Constants.Intake.OUTTAKE_MOTOR_SPEED);
        break;
      case Hold:
        intakeMotor.set(Constants.Intake.HOLD_MOTOR_SPEED);
        break;
      case Idle:
        intakeMotor.set(Constants.Intake.IDLE_MOTOR_SPEED);
        break;
    }
  }

  public void SetIntakeMode(Modes intakeMode) {
    this.intakeMode = intakeMode;
  }
}
