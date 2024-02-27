// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.Constants.Climber.PIDConstants;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX climberMotorRight;
  private TalonFX climberMotorLeft;
  private double targetExtension;
  private double currentExtension;
  private double filterOutput;
  private double percentPower;
  private LinearFilter filter;
  private PIDController climberController;
  private Modes currentMode;
  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber tab");

  public enum Modes {
    PERCENT_CONTROL,
    POSITION_CONTROL,
    ZERO
  }

  /** Creates a new Climber. */
  public ClimberSubsystem() {
    //DEFINE MOTORS
    climberMotorRight = new TalonFX(Climber.Ports.CLIMBER_MOTOR_RIGHT_PORT);
    climberMotorLeft = new TalonFX(Climber.Ports.CLIMBER_MOTOR_LEFT_PORT);

    //MOTOR CONFIG
    climberMotorRight.clearStickyFaults();

    climberMotorRight.setControl(new Follower(climberMotorLeft.getDeviceID(), false));

    climberMotorRight.setNeutralMode(NeutralModeValue.Brake);
    climberMotorRight.setPosition(0);
    climberMotorRight.setInverted(false);
    climberMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    climberMotorLeft.setPosition(0);
    climberMotorLeft.setInverted(false);

    climberController = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);

    //LINEAR FILTER for stator currents and zeroing
    filter = LinearFilter.movingAverage(30);

    currentMode = Modes.ZERO;


    //SHUFFLEBOARD
    climberTab.addDouble("Target Extension", () -> targetExtension);
    climberTab.addDouble("Current Extension", this::getCurrentExtension);
    climberTab.addDouble("Filter Output", () -> filterOutput);
    climberTab.addBoolean("Stator Ecceded?", () -> Math.abs(filterOutput) > Climber.ZERO_STATOR_LIMIT);
    climberTab.addDouble("Current Motor Power", () -> climberMotorRight.get());
    climberTab.addDouble(
        "Stator Current", () -> climberMotorRight.getStatorCurrent().getValueAsDouble());
    climberTab.add("PID Controller", climberController);
    climberTab.addString("Current Mode", () -> currentMode.toString());
    climberTab.addDouble("Motor rotations", () -> climberMotorRight.getPosition().getValueAsDouble());
  }

  //SETTERS
  public void setTargetExtension(double targetExtension) {
    this.targetExtension = targetExtension;
  }

  public void setMode(Modes mode) {
    currentMode = mode;
  }

  public void setPercentPower(double percentPower) {
    this.percentPower = MathUtil.clamp(percentPower, -1, 1);
  }

  //GETTERS
  private double getFilterOutput() {
    return filterOutput;
  }

  public double getTargetExtension() {
    return targetExtension;
  }

  public double getCurrentExtension() {
    return currentExtension;
  }

  private double getCurrentRotations() {
    return climberMotorRight.getPosition().getValueAsDouble();//negate the value to invert the motor
  }

  public double getCurrentExtensionInches() {
    return getCurrentRotations()
        / Climber.CLIMBER_GEAR_RATIO
        * (Climber.SPROCKET_DIAMETER * Math.PI);
  }

  //ROTATION TO INCHES CONVERSION
  public double rotationsToExtensionInches(double rotations) {
    return rotations
        / Climber.CLIMBER_GEAR_RATIO
        * (Climber.SPROCKET_DIAMETER * Math.PI);
  }

  public double extensionInchesToRotations(double inches) {
    return inches 
      * Climber.CLIMBER_GEAR_RATIO 
      / (Climber.SPROCKET_DIAMETER * Math.PI);
  }

  //PERIODICS
  private void positionDrivePeriodic() {
    climberMotorRight.set(
      MathUtil.clamp(climberController.calculate(currentExtension, targetExtension), -1, 1));
  }

  private void percentDrivePeriodic() {
    climberMotorRight.set(percentPower);
  }

  private void zeroPeriodic() {
    if (Math.abs(filterOutput) > Climber.ZERO_STATOR_LIMIT) {
      currentMode = Modes.PERCENT_CONTROL;
      climberMotorRight.set(0);
      climberMotorRight.setPosition(0);
    } else {
      climberMotorRight.set(-0.25);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentExtension = getCurrentExtensionInches();
    filterOutput = filter.calculate(climberMotorRight.getStatorCurrent().getValueAsDouble());

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
