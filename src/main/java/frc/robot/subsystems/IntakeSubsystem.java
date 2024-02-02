// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
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

  private final TalonFX rightIntakeMotor;//LEADER
  private final TalonFX leftIntakeMotor;//FOLLOWER
  private final TalonFX serializerMotor;
  private final DigitalInput noteSensor;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  private Modes intakeMode;

  public enum Modes {
    INTAKE(Intake.INTAKE_MOTOR_SPEED),
    OUTTAKE(Intake.OUTTAKE_MOTOR_SPEED),
    HOLD(Intake.HOLD_MOTOR_SPEED);

    public final double motorSpeed;

    private Modes(double motorSpeed) {
      this.motorSpeed = motorSpeed;
    }
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    rightIntakeMotor = new TalonFX(Intake.Ports.RIGHT_INTAKE_MOTOR_PORT);    
    leftIntakeMotor = new TalonFX(Intake.Ports.LEFT_INTAKE_MOTOR_PORT);
    serializerMotor = new TalonFX(Intake.Ports.SERIALIZER_MOTOR_PORT);

    rightIntakeMotor.clearStickyFaults();
    leftIntakeMotor.clearStickyFaults();
    serializerMotor.clearStickyFaults();

    rightIntakeMotor.setNeutralMode(NeutralModeValue.Brake);    
    leftIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    serializerMotor.setNeutralMode(NeutralModeValue.Brake);


    leftIntakeMotor.setControl(
        new Follower(
            rightIntakeMotor.getDeviceID(),
            false)); // set serializer motor to follow the intake motor and also be inverted

    serializerMotor.setControl(
        new Follower(
            rightIntakeMotor.getDeviceID(),
            Intake.IS_SERIALIZER_INVERTED)); // set serializer motor to follow the intake motor and also be inverted

  
    // Mode to tell the motor what speed to go at
    intakeMode = Modes.HOLD; // default to hold

    // BEAM BRAKE SENSOR - once the beam is broken we need to hold the piece in place.
    noteSensor = new DigitalInput(Intake.Ports.INTAKE_SENSOR_PORT);

  
    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      tab.addDouble("intake voltage", () -> rightIntakeMotor.getMotorVoltage().getValueAsDouble());
      tab.addDouble(
          "Serializer motor voltage", () -> serializerMotor.getMotorVoltage().getValueAsDouble());
      tab.addBoolean("Note Sensor Output", this::getSensorOutput);
      tab.addString("Current Mode", () -> intakeMode.toString());
    }
  }

  public boolean getSensorOutput() {
    return noteSensor.get();
  }

  public void setIntakeMode(Modes intakeMode) {
    this.intakeMode = intakeMode;
  }

  private void setIntakeMotorSpeed() { // using the current mode, set the motor speed
    rightIntakeMotor.set(intakeMode.motorSpeed);
  }

  @Override
  public void periodic() {

    // check for a beam break
    // if (getSensorOutput()) { // if the beam is broken
    //   intakeMode = Modes.HOLD; // set the mode to hold the note
    // }

    setIntakeMotorSpeed();
  }
}
