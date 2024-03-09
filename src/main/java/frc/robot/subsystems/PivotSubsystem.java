// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Pivot;
import frc.robot.Constants.Pivot.Setpoints;
import frc.util.Util;
import java.util.Optional;

public class PivotSubsystem extends SubsystemBase {

  private final TalonFX pivotMotor;

  private final CANcoder pivotCANcoder;

  private PIDController pidController;

  private double targetDegrees;
  private double pidVoltageOutput;
  private double calculatedTargetDegrees;

  private boolean inRange;
  private Pose2d pose;

  private final ShuffleboardTab pivotTab = Shuffleboard.getTab("Pivot");

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    pivotMotor = new TalonFX(Pivot.Ports.PIVOT_MOTOR_PORT);

    pivotCANcoder = new CANcoder(Pivot.Ports.CANCODER_PORT);
    pivotCANcoder.getConfigurator().apply(Pivot.MotorConfigs.CANCODER_CONFIG);

    pivotMotor.getConfigurator().apply(Pivot.MotorConfigs.PIVOT_CONFIG);
    pivotMotor.setInverted(true);
    pivotMotor.clearStickyFaults();
    pivotMotor.set(0);

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    pidController = new PIDController(0.3, 0, 0);

    targetDegrees = 0;
    pidVoltageOutput = 0;

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      pivotTab.addNumber(
          "Current Motor Position", () -> pivotMotor.getPosition().getValueAsDouble());
      pivotTab.addNumber("Current Pivot Angle", this::getCurrentAngle);
      pivotTab.addBoolean("Is at target", this::isAtTargetDegrees);
      pivotTab.addNumber("Motor Error", this::getCurrentError);
      pivotTab.addNumber("PID Error", pidController::getPositionError);
      pivotTab.addNumber("Target Degrees", this::getTargetDegrees);
      pivotTab.addNumber("Applied Voltage", () -> pivotMotor.getMotorVoltage().getValueAsDouble());
      pivotTab.addDouble("PID Voltage Output", () -> pidVoltageOutput);
      pivotTab.addDouble("Calculated Target Angle", () -> calculatedTargetDegrees);
      pivotTab.add(pidController);
    }
  }

  private double getFeedForward() {
    return Math.cos(Math.toRadians(getCurrentAngle())) * Pivot.GRAVITY_VOLTAGE;
  }

  private double getCurrentError() {
    return targetDegrees - getCurrentAngle();
  }

  public double getCurrentAngle() {
    return rotationsToDegrees(pivotCANcoder.getAbsolutePosition().getValueAsDouble());
  }

  public double getTargetDegrees() {
    return targetDegrees;
  }

  public boolean isAtTargetDegrees() {
    return Util.epsilonEquals(getCurrentAngle(), targetDegrees, Pivot.EPSILON);
  }

  public boolean isReadyToShoot(){
    return isAtTargetDegrees() && inRange;
  }

  public void setTargetDegrees(double degrees) {
    this.targetDegrees = MathUtil.clamp(degrees, Setpoints.MINIMUM_ANGLE, Setpoints.MAXIMUM_ANGLE);
  }

  private static double rotationsToDegrees(double rotations) {
    return (rotations * 360);
  }

  public void calculatePivotTargetDegrees(Pose2d pose, double xV, double yV) {
    this.pose = pose;
    double g = Pivot.GRAVITY;
    double x = pose.getX();
    double y = pose.getY();
    double speakerX;
    double speakerY;
    calculatedTargetDegrees = getCurrentAngle();
    Optional<Alliance> color = DriverStation.getAlliance();

    if (color.isPresent() && color.get() == Alliance.Red) {
      speakerX = Pivot.RED_SPEAKER_POSE.getX();
      speakerY = Pivot.RED_SPEAKER_POSE.getY();
    } else {
      speakerX = Pivot.BLUE_SPEAKER_POSE.getX();
      speakerY = Pivot.BLUE_SPEAKER_POSE.getY();
    }
    double distanceToSpeaker = Math.sqrt(Math.pow((x - speakerX), 2) + Math.pow((y - speakerY), 2));

    double d = distanceToSpeaker + Pivot.PIVOT_TO_ROBO_CENTER_LENGTH;
    double h = Pivot.SPEAKER_HEIGHT - Pivot.PIVOT_TO_ROBO_CENTER_HEIGHT;

    // difference between distance to speaker now and after 1 second to find v to speaker
    double velocityToSpeaker =
        distanceToSpeaker
            - Math.sqrt(
                (Math.pow((x - speakerX), 2)
                    + Math.pow((y - speakerY), 2)));

    double v = Pivot.NOTE_SPEED + velocityToSpeaker;

    double interiorMath = (v * v * v * v) - g * ((g * d * d) + (2 * h * v * v));

    if (interiorMath > 0) {
      calculatedTargetDegrees =
          180 / Math.PI * (Math.atan(((v * v) - Math.sqrt(interiorMath)) / (g * d)));
      targetDegrees = calculatedTargetDegrees;
      inRange = true;
    } else {
      inRange = false;
    }
  }

  private boolean inAngleRange(double angle) {
    return angle < Setpoints.MAXIMUM_ANGLE && angle > Setpoints.MINIMUM_ANGLE;
  }

  private boolean currentOrTargetAngleIsUnsafe() {
    return !inAngleRange(getCurrentAngle()) || !inAngleRange(targetDegrees);
  }

  private double computeTargetDegrees() {
    if (currentOrTargetAngleIsUnsafe()) {
      if (getCurrentAngle() < 45) {
        return Setpoints.MINIMUM_SAFE_THRESHOLD;
      }
      return Setpoints.MAXIMUM_SAFE_THRESHOLD;
    }
    return targetDegrees;
  }

  @Override
  public void periodic() {
    double pidOutput = pidController.calculate(getCurrentAngle(), computeTargetDegrees());

    pidVoltageOutput = MathUtil.clamp(pidOutput + getFeedForward(), -10, 10);

    pivotMotor.setVoltage(pidVoltageOutput);
  }
}
