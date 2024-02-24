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
  private final TalonFX pivotMotor;
  private final TalonFX rollerMotorBottom;
  private final TalonFX rollerMotorTop;
  private final TalonFX acceleratorMotor;

  private final CANcoder pivotCANcoder;
  private DigitalInput noteSensor;

  private PIDController pidController;

  private ShooterMode shooterMode;
  private PivotMode pivotMode;

  private double targetDegrees;
  private double pidVoltageOutput;
  private double manualVolatgeOutput;
  private boolean inRange;
  private double mathedTargetDegrees;

  private Pose2d pose;

  private final ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

  public enum ShooterMode {
    INTAKE(Shooter.Modes.INTAKE),
    IDLE(Shooter.Modes.IDLE),
    RAMPING(Shooter.Modes.RAMPING),
    SHOOTING(Shooter.Modes.SHOOTING),
    TARGETLOCK(Shooter.Modes.TARGET_LOCK);

    public final ShooterPowers shooterPowers;

    private ShooterMode(ShooterPowers shooterPowers) {
      this.shooterPowers = shooterPowers;
    }
  }

  public enum PivotMode {
    ANGLE,
    VOLTAGE
  }

  public record ShooterPowers(double roller, double accelerator) {
    public ShooterPowers(double roller, double accelerator) {
      this.roller = roller;
      this.accelerator = accelerator;
    }
  }

  public ShooterSubsystem() {
    // PORTS
    pivotMotor = new TalonFX(Shooter.Ports.PIVOT_MOTOR_PORT);
    rollerMotorTop = new TalonFX(Shooter.Ports.TOP_SHOOTER_MOTOR_PORT);
    // rollerMotorTop.getConfigurator().apply(new TalonFXConfiguration());
    rollerMotorBottom = new TalonFX(Shooter.Ports.BOTTOM_SHOOTER_MOTOR_PORT);
    // rollerMotorBottom.getConfigurator().apply(new TalonFXConfiguration());
    acceleratorMotor = new TalonFX(Shooter.Ports.ACCELERATOR_MOTOR_PORT);
    // acceleratorMotor.getConfigurator().apply(new TalonFXConfiguration());
    noteSensor = new DigitalInput(Shooter.Ports.BEAM_BREAK_SENSOR_PORT);

    pivotCANcoder = new CANcoder(Shooter.Ports.CANCODER_PORT);
    pivotCANcoder.getConfigurator().apply(Shooter.MotorConfigs.CANCODER_CONFIG);

    pivotMotor.getConfigurator().apply(Shooter.MotorConfigs.PIVOT_CONFIG);
    pivotMotor.setInverted(true);
    pivotMotor.clearStickyFaults();
    pivotMotor.set(0);

    rollerMotorTop.clearStickyFaults();
    acceleratorMotor.clearStickyFaults();
    rollerMotorBottom.clearStickyFaults();

    rollerMotorBottom.setControl(new Follower(rollerMotorTop.getDeviceID(), false));

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    acceleratorMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorTop.setNeutralMode(NeutralModeValue.Coast);
    rollerMotorBottom.setNeutralMode(NeutralModeValue.Coast);

    pidController = new PIDController(0.2, 0, 0);

    targetDegrees = 0;
    pidVoltageOutput = 0;
    manualVolatgeOutput = 0;

    shooterMode = ShooterMode.IDLE;
    pivotMode = PivotMode.ANGLE;

    // SHUFFLEBOARD
    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      pivotTab.addNumber(
          "Current Motor Position", () -> pivotMotor.getPosition().getValueAsDouble());
      pivotTab.addNumber("Current Pivot Angle", this::getCurrentAngle);
      pivotTab.addBoolean("Sensor Input", this::isBeamBreakSensorTriggered);
      pivotTab.addBoolean("Is at target", this::isAtTargetDegrees);
      pivotTab.addNumber("Motor Error", this::getCurrentError);
      pivotTab.addNumber("PID Error", pidController::getPositionError);
      pivotTab.addNumber("Target Degrees", this::getTargetDegrees);
      pivotTab.addNumber("Applied Voltage", () -> pivotMotor.getMotorVoltage().getValueAsDouble());
      pivotTab.addDouble("PID Voltage Output", () -> pidVoltageOutput);
      pivotTab.addDouble("Roller Velocity", () -> rollerMotorTop.getVelocity().getValueAsDouble());
      pivotTab.addDouble("Calculated Target Angle", () -> mathedTargetDegrees);
      pivotTab.add(pidController);
      pivotTab.addString("Pivot mode", () -> pivotMode.toString());
    }
  }

  private double getFeedForward() {
    return Math.cos(Math.toRadians(getCurrentAngle())) * Shooter.GRAVITY_VOLTAGE;
  }

  private double getCurrentError() {
    return targetDegrees - getCurrentAngle();
  }

  public double getCurrentAngle() {
    return rotationsToDegrees(pivotCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  private double getTargetDegrees() {
    return targetDegrees;
  }

  public boolean isAtTargetDegrees() {
    return Util.epsilonEquals(getCurrentAngle(), targetDegrees, Shooter.EPSILON);
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

  public void calculatePivotTargetDegrees(Pose2d pose, double xV, double yV) {
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

  public void setTargetDegrees(double degrees) {
    this.targetDegrees =
        MathUtil.clamp(degrees, Setpoints.MINIMUM_SAFE_THRESHOLD, Setpoints.MAXIMUM_SAFE_THRESHOLD);
    this.pivotMode = PivotMode.ANGLE;
  }

  public void setShooterMode(ShooterMode newMode) {
    this.shooterMode = newMode;
  }

  public void setManualVolatgeOutput(double voltage) {
    this.manualVolatgeOutput = voltage;
    this.pivotMode = PivotMode.VOLTAGE;
  }

  private static double degreesToRotations(double angle) {
    return (angle / 360);
  }

  private static double rotationsToDegrees(double rotations) {
    return (rotations * 360);
  }

  // private boolean withinAngleRange(double angle) {
  //   return angle < Shooter.Setpoints.MAXIMUM_ANGLE && angle > Shooter.Setpoints.MINIMUM_ANGLE;
  // }

  // private boolean currentOrTargetAngleUnsafe() {
  //   return !withinAngleRange(getCurrentAngle()) || !withinAngleRange(targetDegrees);
  // }

  private double clampedTargetDegrees() {
    return MathUtil.clamp(targetDegrees, Setpoints.MINIMUM_SAFE_THRESHOLD, Setpoints.MAXIMUM_SAFE_THRESHOLD);
  }

  private void applyPivotMode() {
    if (pivotMode == PivotMode.ANGLE) {
      pivotAnglePeriodic();
    } else {
      pivotVoltagePeriodic();
    }
  }

  private void pivotAnglePeriodic() {
    targetDegrees = clampedTargetDegrees();

    double pidOutput = pidController.calculate(getCurrentAngle(), targetDegrees);

    pidVoltageOutput = MathUtil.clamp(pidOutput + getFeedForward(), -10, 10);

    pivotMotor.setVoltage(pidVoltageOutput);
  }

  private void pivotVoltagePeriodic() {
    pivotMotor.setVoltage(MathUtil.clamp(manualVolatgeOutput + getFeedForward(), -4, 4));
  }

  @Override
  public void periodic() {
    applyPivotMode();

    // shooter motor power
    rollerMotorTop.set(shooterMode.shooterPowers.roller());
    acceleratorMotor.set(shooterMode.shooterPowers.accelerator());
  }
}
