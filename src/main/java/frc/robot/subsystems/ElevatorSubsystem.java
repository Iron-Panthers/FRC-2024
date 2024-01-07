// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// need: have a way for elevator to go up, take in double and goes up
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

/** Add your docs here. */
public class ElevatorSubsystem extends SubsystemBase {

  public enum Modes {
    PERCENT_CONTROL,
    POSITION_CONTROL,
    ZERO
  }

  private Modes currentMode;

  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private TalonFX wristMotor;

  private double currentExtension;
  private double targetExtension;
  private double currentWristAngle;
  private double targetAngle;
  private double elevatorPercentControl;
  private double wristPercentControl;

  private boolean elevatorZeroed;
  private boolean wristZeroed;

  // private ProfiledPIDController extensionController;
  private PIDController wristController;
  // private CANCoder canCoder;

  private final ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

  private final ShuffleboardTab wristTab = Shuffleboard.getTab("Wrist");

  private double elevatorFilterOutput;

  private double wristFilterOutput;

  // private DigitalInput proxySensor;

  // add soft limits - check 2022 frc code

  // stator limits
  private LinearFilter elevatorFilter;
  private LinearFilter wristFilter;

  public static record ElevatorState(double extension, double angle) {}

  public ElevatorSubsystem() {

    currentMode = Modes.ZERO;

    leftMotor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_LEFT_MOTOR_PORT);
    rightMotor = new TalonFX(Constants.Elevator.Ports.ELEVATOR_RIGHT_MOTOR_PORT);
    wristMotor = new TalonFX(Constants.Elevator.Ports.WRIST_MOTOR_PORT);

    leftMotor.follow(rightMotor);

    // extensionController = new PIDController(0.1, 0.03, 0.035);
    wristController = new PIDController(0.1, 0, 0);
    // canCoder = new CANCoder(Elevator.Ports.CANCODER);
    // proxySensor = new DigitalInput(0);

    // extensionController.setTolerance(0.25, 0.05);

    rightMotor.setSelectedSensorPosition(0);
    leftMotor.setSelectedSensorPosition(0);
    wristMotor.setSelectedSensorPosition(0);

    currentExtension = 0.0;
    targetExtension = 0.0;
    currentWristAngle = 0.0;
    targetAngle = 0.0;
    elevatorPercentControl = 0.0;
    wristPercentControl = 0.0;

    elevatorZeroed = false;
    wristZeroed = false;

    rightMotor.configFactoryDefault();
    leftMotor.configFactoryDefault();
    wristMotor.configFactoryDefault();

    rightMotor.clearStickyFaults();
    leftMotor.clearStickyFaults();
    wristMotor.clearStickyFaults();

    rightMotor.setInverted(true);
    leftMotor.setInverted(InvertType.FollowMaster);
    // wristMotor.setInverted(false);
    wristMotor.setSensorPhase(true);

    rightMotor.configOpenloopRamp(.5);

    rightMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.setNeutralMode(NeutralMode.Brake);
    wristMotor.setNeutralMode(NeutralMode.Brake);

    rightMotor.configForwardSoftLimitThreshold(
        extensionInchesToTicks(Constants.Elevator.MAX_EXTENSION_INCHES), 20);
    rightMotor.configReverseSoftLimitThreshold(
        extensionInchesToTicks(Elevator.MIN_EXTENSION_INCHES), 20);
    wristMotor.configReverseSoftLimitThreshold(angleToTicks(Elevator.MIN_ANGLE_DEGREES));
    wristMotor.configForwardSoftLimitThreshold(angleToTicks(Elevator.MAX_ANGLE_DEGREES));

    rightMotor.configForwardSoftLimitEnable(true, 20);
    rightMotor.configReverseSoftLimitEnable(true, 20);
    wristMotor.configForwardSoftLimitEnable(true);
    // wristMotor.configReverseSoftLimitEnable(true);

    rightMotor.configMotionAcceleration(30000, 30);
    rightMotor.configMotionCruiseVelocity(15000, 30);

    rightMotor.config_kP(0, 0.08);
    rightMotor.config_kD(0, 0);
    rightMotor.config_kI(0, 0);
    rightMotor.config_kF(0, 0.0);
    rightMotor.configNeutralDeadband(0.001);
    // canCoder.configMagnetOffset(Elevator.ANGULAR_OFFSET);

    // canCoder.configSensorDirection(true);

    // canCoder.setPositionToAbsolute(10); // ms

    elevatorFilter = LinearFilter.movingAverage(30);
    wristFilter = LinearFilter.movingAverage(30);

    wristTab.addDouble("Wrist Target Angle", () -> targetAngle);
    wristTab.addDouble("Wrist Current Angle", () -> currentWristAngle);
    wristTab.addDouble("filter output", () -> wristFilterOutput);
    wristTab.addDouble("percent control", () -> wristPercentControl);
    wristTab.addBoolean("is zeroed", () -> wristZeroed);
    wristTab.add("Wrist PID", wristController);
    wristTab.addString("mode", () -> currentMode.toString());
    elevatorTab.addDouble("Elevator Target Extension", () -> targetExtension);
    elevatorTab.addDouble("Elevator Current Extension", () -> currentExtension);
    elevatorTab.addDouble("Elevator Output", rightMotor::getMotorOutputPercent);
    elevatorTab.addDouble("percent control", () -> elevatorPercentControl);
    elevatorTab.addDouble("velocity", rightMotor::getSelectedSensorVelocity);
    elevatorTab.addDouble("filter output", () -> elevatorFilterOutput);
    elevatorTab.addDouble("Stator current", rightMotor::getStatorCurrent);
    elevatorTab.addBoolean("is zeroed", () -> elevatorZeroed);
    elevatorTab.addString("mode", () -> currentMode.toString());
  }

  public void setMode(Modes mode) {
    currentMode = mode;
  }

  public Modes getMode() {
    return currentMode;
  }

  public static double extensionInchesToTicks(double inches) {
    return (Elevator.FALCON_CPR * inches)
        / (Elevator.CARRIAGE_RATIO
            * (Elevator.ELEVATOR_SPROCKET_DIAMETER_INCHES * Math.PI)
            * Elevator.ELEVATOR_GEAR_RATIO);
  }

  public double ticksToExtensionInches(double ticks) {
    return Elevator.CARRIAGE_RATIO
        * (Elevator.ELEVATOR_SPROCKET_DIAMETER_INCHES * Math.PI)
        * ((ticks / Elevator.FALCON_CPR) * Elevator.ELEVATOR_GEAR_RATIO);
  }

  private double getCurrentTicks() {
    return rightMotor.getSelectedSensorPosition();
  }

  public void setTargetExtensionInches(double targetExtension) {
    this.targetExtension = targetExtension;
  }

  public void setTargetState(ElevatorState targetState) {
    targetExtension = targetState.extension();
    targetAngle = targetState.angle();
  }

  public void setPercentControl(double elevatorPercentControl, double wristPercentControl) {
    this.elevatorPercentControl = elevatorPercentControl;
    this.wristPercentControl = wristPercentControl;
  }

  public double getElevatorPercentControl() {
    return elevatorPercentControl;
  }

  public double getWristPercentControl() {
    return wristPercentControl;
  }

  public double getTargetExtension() {
    return targetExtension;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getCurrentExtensionInches() {
    return ticksToExtensionInches(getCurrentTicks());
  }

  public double getCurrentAngleDegrees() {
    // 1:128 ratio
    return (wristMotor.getSelectedSensorPosition() * 360) / (128 * 2048);
    // return canCoder.getAbsolutePosition();
  }

  private double angleToTicks(double angle) {
    return angle / 360 * 128 * 2048;
  }

  private double ticksToAngle(double ticks) {
    return ticks / 2048 / 128 * 360;
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  public void setNotZeroed() {
    elevatorZeroed = false;
    wristZeroed = false;
  }

  private double applySlowZoneToElevatorPercent(double elevatorPercentControl) {
    if ((currentExtension > (Elevator.MAX_EXTENSION_INCHES - 7))
        || (currentExtension < (Elevator.MIN_EXTENSION_INCHES + 7))) {
      return MathUtil.clamp(elevatorPercentControl * 0.5, -1, 1);
    }
    return MathUtil.clamp(elevatorPercentControl, -1, 1);
  }

  private void percentDrivePeriodic() {
    if (elevatorFilterOutput > Elevator.STATOR_LIMIT) {
      rightMotor.set(TalonFXControlMode.PercentOutput, 0);
    } else {
      rightMotor.set(
          TalonFXControlMode.PercentOutput,
          applySlowZoneToElevatorPercent(elevatorPercentControl) + 0.02);
    }

    // rightMotor.set(TalonFXControlMode.MotionMagic, extensionInchesToTicks(targetExtension));
    if (wristFilterOutput > Elevator.STATOR_LIMIT) {
      wristMotor.set(TalonFXControlMode.PercentOutput, 0);
    } else {
      wristMotor.set(TalonFXControlMode.PercentOutput, MathUtil.clamp(wristPercentControl, -0.75, 0.75));
    }
  }

  private void positionDrivePeriodic() {
    // if (filterOutput < statorCurrentLimit) {
    //   double motorPower = extensionController.calculate(currentExtension, targetExtension);
    //   final double gravityOffset = calculateElevatorGravityOffset();

    //   rightMotor.set(
    //       TalonFXControlMode.PercentOutput, MathUtil.clamp(motorPower + gravityOffset, -0.8,
    // 0.8));
    // } else {
    //   rightMotor.set(TalonFXControlMode.PercentOutput, 0);
    // }
    if (elevatorFilterOutput > Elevator.STATOR_LIMIT) {
      rightMotor.set(TalonFXControlMode.PercentOutput, 0);
    } else {
      rightMotor.set(TalonFXControlMode.MotionMagic, extensionInchesToTicks(targetExtension));
    }

    if (wristFilterOutput > Elevator.STATOR_LIMIT) {
      wristMotor.set(TalonFXControlMode.PercentOutput, 0);
    } else {
      wristMotor.set(
          TalonFXControlMode.PercentOutput,
          MathUtil.clamp(wristController.calculate(currentWristAngle, targetAngle), -0.75, 0.75));
    }
  }

  private void zeroPeriodic() {

    rightMotor.configForwardSoftLimitEnable(false, 20);
    rightMotor.configReverseSoftLimitEnable(false, 20);

    rightMotor.set(TalonFXControlMode.PercentOutput, Elevator.ZERO_MOTOR_POWER);

    if (elevatorFilterOutput > Elevator.ZERO_STATOR_LIMIT) {
      rightMotor.setSelectedSensorPosition(0);
      leftMotor.setSelectedSensorPosition(0);

      rightMotor.set(TalonFXControlMode.PercentOutput, 0);

      rightMotor.configForwardSoftLimitEnable(true, 20);
      rightMotor.configReverseSoftLimitEnable(true, 20);

      targetExtension = Elevator.MIN_EXTENSION_INCHES;

      elevatorZeroed = true;
    }
    if (elevatorZeroed) {
      wristMotor.configForwardSoftLimitEnable(false, 20);
      wristMotor.configReverseSoftLimitEnable(false);
      wristMotor.set(TalonFXControlMode.PercentOutput, Elevator.ZERO_MOTOR_POWER);
      if (wristFilterOutput > Elevator.ZERO_STATOR_LIMIT) {
        wristMotor.setSelectedSensorPosition(0);

        wristMotor.set(TalonFXControlMode.PercentOutput, 0);

        wristMotor.configForwardSoftLimitEnable(true);
        wristMotor.configReverseSoftLimitEnable(true);

        wristZeroed = true;
      }
    }
    if (wristZeroed && elevatorZeroed) {
      currentMode = Modes.POSITION_CONTROL;
    }
  }

  @Override
  public void periodic() {
    currentExtension = getCurrentExtensionInches();
    currentWristAngle = getCurrentAngleDegrees();
    elevatorFilterOutput = elevatorFilter.calculate(rightMotor.getStatorCurrent());
    wristFilterOutput = wristFilter.calculate(wristMotor.getStatorCurrent());

    switch (currentMode) {
      case ZERO:
        zeroPeriodic();
        break;
      case POSITION_CONTROL:
        positionDrivePeriodic();
        break;
      case PERCENT_CONTROL:
        percentDrivePeriodic();
        break;
    }
  }
}
