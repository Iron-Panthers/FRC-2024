// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX climberMotor;
  private double targetExtension;
  private double currentExtension;
  private double filterOutput;
  private double percentPower;
  private LinearFilter filter;
  private PIDController climberController;
  private Modes currentMode;
  private TalonFXConfiguration climberConfig;
  private MotionMagicConfigs motionMagicConfigs;
  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);// create a Motion Magic request, voltage output
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber tab");

  public enum Modes {
    PERCENT_CONTROL,
    POSITION_CONTROL,
    ZERO
  }

  /** Creates a new Climber. */
  public ClimberSubsystem() {
    climberMotor = new TalonFX(Climber.Ports.CLIMBER_MOTOR_PORT);
    climberConfig = new TalonFXConfiguration();
    motionMagicConfigs = climberConfig.MotionMagic;

    climberMotor.clearStickyFaults();
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
    climberMotor.setPosition(0);

    //Motion Magic Configuration
    climberMotor.getConfigurator().apply(new TalonFXConfiguration()); // Applies factory defaults

    motionMagicConfigs.MotionMagicAcceleration = 1000; // These values are in RPS
    motionMagicConfigs.MotionMagicCruiseVelocity = 1000;
    climberConfig.Slot0.kS = 0;
    climberConfig.Slot0.kV = 0;
    climberConfig.Slot0.kA = 0;
    climberConfig.Slot0.kP = 0;
    climberConfig.Slot0.kI = 0;
    climberConfig.Slot0.kD = 0;

    climberMotor.getConfigurator().apply(climberConfig);//put the configuration on the motor

    filter = LinearFilter.movingAverage(30);

    currentMode = Modes.ZERO;

    climberTab.addDouble("Target Extension", () -> targetExtension);
    climberTab.addDouble("Current Extension", () -> currentExtension);
    climberTab.addDouble("Filter Output", () -> filterOutput);
    climberTab.addDouble("Current Motor Power", () -> climberMotor.get());
    climberTab.addDouble(
        "Stator Current", () -> climberMotor.getStatorCurrent().getValueAsDouble());
    climberTab.add("PID Controller", climberController);
    climberTab.addString("Current Mode", () -> currentMode.toString());
  }

  public void setTargetExtension(double targetExtension) {
    this.targetExtension = targetExtension;
  }

  public void setMode(Modes mode) {
    currentMode = mode;
  }

  public void setPercentPower(double percentPower) {
    this.percentPower = MathUtil.clamp(percentPower, -1, 1);
  }

  public double getFilterOutput() {
    return filterOutput;
  }

  public double getTargetExtension() {
    return targetExtension;
  }

  public double getCurrentExtension() {
    return currentExtension;
  }

  public double getCurrentTicks() {
    return climberMotor.getPosition().getValueAsDouble();
  }
  
  public double currentExtensionInchesToTicks() {
    return getCurrentExtension() * Climber.FALCON_CPR * Climber.CLIMBER_GEAR_RATIO;
  }

  public double ticksToExtensionInches() {
    return getCurrentTicks() / Climber.FALCON_CPR / Climber.CLIMBER_GEAR_RATIO;
  }

  public double extensionInchesToRotations(double inches) {
    return inches * Climber.CLIMBER_GEAR_RATIO;
  }


  private void positionDrivePeriodic() {
    climberMotor.setControl(m_request.withPosition(extensionInchesToRotations(targetExtension)));
        //MathUtil.clamp(climberController.calculate(currentExtension, targetExtension), -1, 1));
  }

  private void percentDrivePeriodic() {
    climberMotor.set(percentPower);
  }

  private void zeroPeriodic() {
    if (filterOutput > Climber.ZERO_STATOR_LIMIT) {
      currentMode = Modes.PERCENT_CONTROL;
      climberMotor.set(0);
      climberMotor.setPosition(0);
    } else {
      climberMotor.set(0.25);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentExtension = ticksToExtensionInches();
    filterOutput = filter.calculate(filterOutput);

    switch (currentMode) {
      case PERCENT_CONTROL:
        percentDrivePeriodic();
        break;
      case POSITION_CONTROL:
        positionDrivePeriodic();
        break;
      case ZERO:
        zeroPeriodic();
        break;
    }
  }
}
