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
import frc.robot.Constants.Config;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.IntakeSubsystemModeSettings;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TalonFX serializerMotor;
  public final ShooterSubsystem shooterSubsystem;
  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");
  private final DigitalInput noteSensor;
  private Modes intakeMode;
  private Modes pastMode; 

  public enum Modes {
    INTAKE(Intake.INTAKE_MODE_SETTINGS),
    HOLD(Intake.HOLD_MODE_SETTINGS),
    REVERSE(Intake.REVERSE_MODE_SETTINGS);

    public final IntakeSubsystemModeSettings modeSettings;

    private Modes(IntakeSubsystemModeSettings modeSettings) {
      this.modeSettings = modeSettings;
    }
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(ShooterSubsystem shooterSubsystem) {

    intakeMotor = new TalonFX(Intake.Ports.INTAKE_MOTOR_PORT);
    serializerMotor = new TalonFX(Intake.Ports.SERIALIZER_MOTOR_PORT);
    noteSensor = new DigitalInput(Intake.Ports.INTAKE_SENSOR_PORT);
    this.shooterSubsystem = shooterSubsystem;
    intakeMotor.clearStickyFaults();
    serializerMotor.clearStickyFaults();

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    serializerMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setInverted(true);
    serializerMotor.setInverted(true);

    // Mode to tell the motor what speed to go at
    intakeMode = Modes.HOLD; // default to hold

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      tab.addDouble("intake voltage", () -> intakeMotor.getMotorVoltage().getValueAsDouble());
      tab.addDouble(
          "Serializer motor voltage", () -> serializerMotor.getMotorVoltage().getValueAsDouble());
      tab.addString("Current Mode", () -> intakeMode.toString());
    }
  }

  // SETTERS

  public void setIntakeMode(Modes intakeMode) {
    this.intakeMode = intakeMode;
  }
  
  public boolean isBeamBreakSensorTriggered() {
    // if is triggered return true
    return !noteSensor.get();
  }

  public boolean areTwoNotesInRobot(){
    return !(isBeamBreakSensorTriggered() && shooterSubsystem.isBeamBreakSensorTriggered());
  }

  private void resolvePenaltyHazard(){
    if (areTwoNotesInRobot()){
        setIntakeMode(intakeMode.REVERSE);
    }
  }

  private Modes getIntakeMode(){
    return intakeMode;
  }

  private void setMotorSpeeds() { // using the current mode, set the motor speed
    if (areTwoNotesInRobot()){
      resolvePenaltyHazard();
    }
    intakeMotor.set(intakeMode.modeSettings.INTAKE_MOTOR_SPEED);
    serializerMotor.set(intakeMode.modeSettings.SERIALIZER_MOTOR_SPEED);
  }

  @Override
  public void periodic() {
    setMotorSpeeds();
  }
}
