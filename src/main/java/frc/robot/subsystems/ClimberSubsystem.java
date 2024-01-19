// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX climberMotor;
  private double targetExtension;
  private double currentExtension;
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber tab");

  public enum Modes {
    EXTENDED,
    RETRACTED
  }
  
  /** Creates a new Climber. */
  public ClimberSubsystem() {
    climberMotor = new TalonFX(Climber.Ports.CLIMBER_MOTOR_PORT);
    climberTab.addDouble("Target Extension", () -> targetExtension);
    climberTab.addDouble("Current Extension", () -> currentExtension);

    climberMotor.clearStickyFaults();
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
    climberMotor.setPosition(0);

    climberMotor.getConfigurator().apply(new TalonFXConfiguration()); // Applies factory defaults
  }

  public void setTargetExtension(double targetExtension) {
    this.targetExtension = targetExtension;
  }

  public double getTargetExtension() {
    return targetExtension;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
