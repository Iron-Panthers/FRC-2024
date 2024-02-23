// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.Shooter.Setpoints;
import frc.util.Util;
import java.util.Optional;

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
  private double mathedTargetDegrees;

  private Pose2d pose;

  private final ShuffleboardTab WristTab = Shuffleboard.getTab("Wrist");

  public enum ShooterMode {
    Intake(Shooter.Modes.INTAKE),
    Idle(Shooter.Modes.IDLE),
    Ramping(Shooter.Modes.RAMPING),
    Shooting(Shooter.Modes.SHOOTING),
    TargetLock(Shooter.Modes.TARGET_LOCK);

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
    wristMotor = new TalonFX(Shooter.Ports.PIVOT_MOTOR_PORT);
    rollerMotorTop = new TalonFX(Shooter.Ports.TOP_SHOOTER_MOTOR_PORT);
    // rollerMotorTop.getConfigurator().apply(new TalonFXConfiguration());
    rollerMotorBottom = new TalonFX(Shooter.Ports.BOTTOM_SHOOTER_MOTOR_PORT);
    // rollerMotorBottom.getConfigurator().apply(new TalonFXConfiguration());
    acceleratorMotor = new TalonFX(Shooter.Ports.ACCELERATOR_MOTOR_PORT);
    // acceleratorMotor.getConfigurator().apply(new TalonFXConfiguration());
    noteSensor = new DigitalInput(Shooter.Ports.BEAM_BREAK_SENSOR_PORT);

    wristCANcoder.getConfigurator().apply(Shooter.MotorConfigs.CANCODER_CONFIG);

    wristMotor.getConfigurator().apply(Shooter.MotorConfigs.PIVOT_CONFIG);
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

    pidController = new PIDController(0.2, 0, 0);

    targetDegrees = 0;
    pidOutput = 0;
    wristPower = 0;

    mode = ShooterMode.Idle;

    // SHUFFLEBOARD
    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      WristTab.addNumber(
          "Current Motor Position", () -> wristMotor.getPosition().getValueAsDouble());
      WristTab.addNumber("Current motor angle", this::getCurrentAngle);
      WristTab.addBoolean("Sensor Input", this::isBeamBreakSensorTriggered);
      WristTab.addNumber("pid Power", () -> pidOutput);
      WristTab.addBoolean("Is at target", this::isAtTargetDegrees);
      WristTab.addNumber("Error", this::getCurrentError);
      WristTab.addNumber("target", this::getTargetDegrees);
      WristTab.addNumber("Error PID", pidController::getPositionError);
      WristTab.addNumber("Applied Voltage", () -> wristMotor.getMotorVoltage().getValueAsDouble());
      WristTab.addDouble("pivot voltage", () -> wristPower);
      WristTab.addDouble("Roller Velocity", () -> rollerMotorTop.getVelocity().getValueAsDouble());
      WristTab.addDouble("Math angle", () -> mathedTargetDegrees);
      WristTab.add(pidController);
    }
  }

  // GETTERS

  private boolean isRedAlliance() {
    Optional<Alliance> color = DriverStation.getAlliance();
    return color.isPresent() && color.get() == Alliance.Red;
  }

  private double getFeedForward() {
    return Math.cos(Math.toRadians(getCurrentAngle())) * Shooter.GRAVITY_VOLTAGE;
  }

  private double getCurrentError() {
    return targetDegrees - getCurrentAngle();
  }

  public double getCurrentAngle() {
    return Util.normalizeDegrees(rotationsToDegrees(wristMotor.getPosition().getValue()));
  }

  private double getTargetDegrees() {
    return targetDegrees;
  }

  public boolean isAtTargetDegrees() {
    return Math.abs(getCurrentError()) < 1;
  }

  public boolean isShooterUpToSpeed() {
    return rollerMotorBottom.getVelocity().getValueAsDouble() >= Shooter.NOTE_SPEED
        && rollerMotorTop.getVelocity().getValueAsDouble() >= Shooter.NOTE_SPEED;
  }

  public boolean isBeamBreakSensorTriggered() {
    // if is triggered return true
    return !noteSensor.get();
  }

  public boolean isReadyToShoot() {
    return isAtTargetDegrees() && isBeamBreakSensorTriggered() && isShooterUpToSpeed();
  }

  // returns wheather or not a change was needed
  public boolean prepareForIntake() {
    if (getCurrentAngle() > 20) {
      setTargetDegrees(20);
      return false;
    }
    return true;
  }

  public void stopAccelerator() {
    acceleratorMotor.set(0);
  }

  public void calculateWristTargetDegrees(Pose2d pose, double xV, double yV) {
    this.pose = pose;
    double g = Shooter.GRAVITY;
    double x = pose.getX();
    double y = pose.getY();
    double speakerX;
    double speakerY;
    mathedTargetDegrees = getCurrentAngle();
    Optional<Alliance> color = DriverStation.getAlliance();

    if (color.isPresent() && color.get() == Alliance.Red) {
      speakerX = Shooter.RED_SPEAKER_POSE.getX();
      speakerY = Shooter.RED_SPEAKER_POSE.getY();
    } else {
      speakerX = Shooter.BLUE_SPEAKER_POSE.getX();
      speakerY = Shooter.BLUE_SPEAKER_POSE.getY();
    }
    double distanceToSpeaker = Math.sqrt(Math.pow((x - speakerX), 2) + Math.pow((y - speakerY), 2));

    double d = distanceToSpeaker - Shooter.PIVOT_TO_ROBO_CENTER_LENGTH;
    double h = Shooter.SPEAKER_HEIGHT - Shooter.PIVOT_TO_ROBO_CENTER_HEIGHT;

    // difference between distance to speaker now and after 1 second to find v to speaker
    double velocityToSpeaker =
        distanceToSpeaker
            - Math.sqrt((Math.pow((x + xV - speakerX), 2) + Math.pow((y + yV - speakerY), 2)));

    System.out.println(velocityToSpeaker);
    double v = Shooter.NOTE_SPEED + velocityToSpeaker;

    double interiorMath = (v * v * v * v) - g * ((g * d * d) + (2 * h * v * v));

    if (interiorMath > 0) {
      mathedTargetDegrees =
          180 / Math.PI * (Math.atan(((v * v) - Math.sqrt(interiorMath)) / (g * d)));
      targetDegrees = mathedTargetDegrees;
      inRange = true;
    } else {
      inRange = false;
    }
  }

  // SETTERS
  public void setTargetDegrees(double degrees) {
    this.targetDegrees =
        MathUtil.clamp(degrees, Setpoints.MINIMUM_SAFE_THRESHOLD, Setpoints.MAXIMUM_SAFE_THRESHOLD);
  }

  public void setShooterMode(ShooterMode newMode) {
    this.mode = newMode;
  }

  public void setPivotVoltage(double voltage) {
    wristMotor.setVoltage(MathUtil.clamp(voltage + getFeedForward(), -4, 4));
  }

  private static double degreesToRotations(double angle) {
    return (angle / 360);
  }

  private static double rotationsToDegrees(double rotations) {
    return (rotations * 360);
  }

  private boolean withinAngleRange(double angle) {
    return angle < Shooter.Setpoints.MAXIMUM_ANGLE && angle > Shooter.Setpoints.MINIMUM_ANGLE;
  }

  private boolean currentOrTargetAngleUnsafe() {
    return !withinAngleRange(getCurrentAngle()) || !withinAngleRange(targetDegrees);
  }

  private double computePivotGoal() {
    if (currentOrTargetAngleUnsafe()) {
      if (getCurrentAngle() < 45) {
        return Setpoints.MINIMUM_SAFE_THRESHOLD;
      }
      return Setpoints.MAXIMUM_SAFE_THRESHOLD;
    }
    return targetDegrees;
  }

  @Override
  public void periodic() {
    // wrist motor power

    pidOutput = pidController.calculate(getCurrentAngle(), computePivotGoal());

    wristPower = MathUtil.clamp(pidOutput + getFeedForward(), -10, 10);

    wristMotor.setVoltage(wristPower);

    // shooter motor power
    rollerMotorTop.set(mode.modeSettings.roller());
    acceleratorMotor.set(mode.modeSettings.accelerator());
  }
}
