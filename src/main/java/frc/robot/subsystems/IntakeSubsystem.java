// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Config;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TalonFX serializerMotor;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  private final DigitalInput noteSensor;

  private Modes intakeMode;

  public enum Modes {
    INTAKE,
    OUTTAKE,
    HOLD,
    IDLE
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);
    serializerMotor = new TalonFX(Constants.Intake.SERIALIZER_MOTOR_PORT);

    intakeMotor.clearStickyFaults();
    serializerMotor.clearStickyFaults();

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    serializerMotor.setNeutralMode(NeutralModeValue.Brake);

    serializerMotor.setControl(new Follower(intakeMotor.getDeviceID(), true));//set serializer motor to follow the intake motor and also be inverted

    // Mode to tell the motor what speed to go at
    intakeMode = Modes.HOLD; // default to hold

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
      intakeMode = Modes.HOLD; // set the mode to hold the note
    }

    setIntakeMotorSpeed();
  }

  public void setIntakeMotorSpeed() { // using the current mode, set the motor speed
    switch (intakeMode) {
      case INTAKE:
        intakeMotor.set(Constants.Intake.INTAKE_MOTOR_SPEED);
        break;
      case OUTTAKE:
        intakeMotor.set(Constants.Intake.OUTTAKE_MOTOR_SPEED);
        break;
      case HOLD:
        intakeMotor.set(Constants.Intake.HOLD_MOTOR_SPEED);
        break;
      case IDLE:
        intakeMotor.set(Constants.Intake.IDLE_MOTOR_SPEED);
        break;
    }
  }

  public void setIntakeMode(Modes intakeMode) {
    this.intakeMode = intakeMode;
  }
}
