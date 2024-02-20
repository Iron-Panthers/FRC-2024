// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;
  private final TalonFX rollerMotorBottom;
  private final TalonFX rollerMotorTop;
  private final TalonFX acceleratorMotor;

  private final CANcoder wristCANcoder = new CANcoder(Shooter.Ports.CANCODER_PORT);
  private DigitalInput noteSensor;

  private PIDController pidController;

  private ShooterMode mode;
  private double targetDegrees;
  private double wristPower;
  private double pidOutput;
  private boolean inRange;

  private final ShuffleboardTab WristTab = Shuffleboard.getTab("Wrist");

  public enum ShooterMode {
    Intake(Shooter.INTAKE_SHOOTER_MODE_CONFIGS),
    Idle(Shooter.IDLE_SHOOTER_MODE_CONFIGS),
    Ramping(Shooter.RAMPING_SHOOTER_MODE_CONFIGS),
    Shooting(Shooter.SHOOTING_SHOOTER_MODE_CONFIGS);

    public final ShooterPowers modeSettings;

    private ShooterMode(ShooterPowers modeSettings) {
      this.modeSettings = modeSettings;
    }
  }

  public record ShooterPowers(double roller, double accelerator) {
    public ShooterPowers(double roller, double accelerator) {
      this.roller = roller;
      this.accelerator = accelerator;
    }
  }

  public ShooterSubsystem() {
    // PORTS
    wristMotor = new TalonFX(Shooter.Ports.WRIST_MOTOR_PORT);
    rollerMotorTop = new TalonFX(Shooter.Ports.TOP_SHOOTER_MOTOR_PORT);
    // rollerMotorTop.getConfigurator().apply(new TalonFXConfiguration());
    rollerMotorBottom = new TalonFX(Shooter.Ports.BOTTOM_SHOOTER_MOTOR_PORT);
    // rollerMotorBottom.getConfigurator().apply(new TalonFXConfiguration());
    acceleratorMotor = new TalonFX(Shooter.Ports.ACCELERATOR_MOTOR_PORT);
    // acceleratorMotor.getConfigurator().apply(new TalonFXConfiguration());
    noteSensor = new DigitalInput(Shooter.Ports.BEAM_BREAK_SENSOR_PORT);

    // WRIST CONFIG
    CANcoderConfiguration wristCANcoderConfig = new CANcoderConfiguration();
    wristCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    wristCANcoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue
            .Clockwise_Positive; // counter clockwise is default, false is counter clockwise
    wristCANcoderConfig.MagnetSensor.MagnetOffset = Shooter.Measurements.WRIST_CANCODER_OFFSET;
    wristCANcoder.getConfigurator().apply(wristCANcoderConfig);

    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.Feedback.FeedbackRemoteSensorID = wristCANcoder.getDeviceID();
    wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    wristMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    wristMotorConfig.Feedback.RotorToSensorRatio = Shooter.Measurements.WRIST_GEAR_RATIO;
    wristMotorConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(0.25);
    wristMotorConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(0);
    wristMotorConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    wristMotorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    wristMotorConfig.Voltage.withPeakForwardVoltage(1.5);
    wristMotorConfig.Voltage.withPeakReverseVoltage(-1.5);
    wristMotor.getConfigurator().apply(wristMotorConfig);
    wristMotor.setInverted(true);
    wristMotor.clearStickyFaults();
    wristMotor.set(0);

    rollerMotorTop.clearStickyFaults();
    acceleratorMotor.clearStickyFaults();
    rollerMotorBottom.clearStickyFaults();

    rollerMotorBottom.setControl(new Follower(rollerMotorTop.getDeviceID(), false));

    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    acceleratorMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorTop.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorBottom.setNeutralMode(NeutralModeValue.Brake);

    // PID
    pidController = new PIDController(0.2, 0, 0);

    targetDegrees = 0;
    pidOutput = 0;
    wristPower = 0;

    mode = ShooterMode.Idle;

    // SHUFFLEBOARD
    WristTab.addNumber("Current Motor Position", () -> wristMotor.getPosition().getValueAsDouble());
    WristTab.addNumber("Current motor angle", this::getCurrentAngle);
    WristTab.addBoolean("Sensor Input", this::isBeamBreakSensorTriggered);
    WristTab.addNumber("pid Power", () -> pidOutput);
    WristTab.addBoolean("Is at target", this::isAtTargetDegrees);
    WristTab.addNumber("Error", this::getCurrentError);
    WristTab.addNumber("target", this::getTargetDegrees);
    WristTab.addNumber("Error PID", pidController::getPositionError);
    WristTab.addNumber("Applied Voltage", () -> wristMotor.getMotorVoltage().getValueAsDouble());
    WristTab.addDouble("pivot voltage", () -> wristPower);
    WristTab.add(pidController);
  }

  // GETTERS
  private double getFeedForward() {
    return Math.cos(Math.toRadians(getCurrentAngle())) * Shooter.GRAVITY_VOLTAGE;
  }

  private double getCurrentError() {
    return targetDegrees - getCurrentAngle();
  }

  private double getCurrentAngle() {
    return rotationsToDegrees(wristMotor.getPosition().getValue());
  }

  private double getTargetDegrees() {
    return targetDegrees;
  }

  public boolean isAtTargetDegrees() {
    return Math.abs(getCurrentError()) < 1;
  }

  public boolean isShooterUpToSpeed() {
    return rollerMotorBottom.getVelocity().getValueAsDouble() >= Shooter.Measurements.NOTE_SPEED
        && rollerMotorTop.getVelocity().getValueAsDouble() >= Shooter.Measurements.NOTE_SPEED;
  }

  public boolean isBeamBreakSensorTriggered() {
    // if is triggered return true
    return !noteSensor.get();
  }

  public boolean isReadyToShoot() {
    return inRange && isAtTargetDegrees() && isBeamBreakSensorTriggered() && isShooterUpToSpeed();
  }

  // returns wheather or not a change was needed
  public boolean prepareForIntake() {
    if (getCurrentAngle() > 20) {
      setTargetDegrees(20);
      return false;
    }
    return true;
  }

  public void stopAccelerator(){
    acceleratorMotor.set(0);
  }

  // SETTERS
  public void setTargetDegrees(double degrees) {
    this.targetDegrees = degrees;
  }

  public void setShooterMode(ShooterMode newMode) {
    this.mode = newMode;
  }

  private static double degreesToRotations(double angle) {
    return (angle / 360);
  }

  private static double rotationsToDegrees(double rotations) {
    return (rotations * 360);
  }

  @Override
  public void periodic() {
    // wrist motor power
    pidOutput = pidController.calculate(getCurrentAngle(), targetDegrees);

    wristPower = MathUtil.clamp(pidOutput + getFeedForward(), -5, 5);

    wristMotor.setVoltage(wristPower);

    // shooter motor power
    rollerMotorTop.set(mode.modeSettings.roller());
    acceleratorMotor.set(mode.modeSettings.accelerator());
  }
}
