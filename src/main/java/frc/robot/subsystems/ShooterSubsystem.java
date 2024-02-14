// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX wristMotor;
  private TalonFX rollerMotorBottom;
  private TalonFX rollerMotorTop;
  private TalonFX acceleratorMotor;
  private PIDController pidController;
  private double targetDegrees;
  private double wristMotorPower;
  private Pose2d pose;
  private DigitalInput noteSensor;
  private boolean inRange;
  private final ShuffleboardTab WristTab = Shuffleboard.getTab("Wrist");

  public ShooterSubsystem() {
    wristMotor = new TalonFX(Shooter.Ports.WRIST_MOTOR_PORT);
    rollerMotorTop = new TalonFX(Shooter.Ports.TOP_SHOOTER_MOTOR_PORT);
    rollerMotorBottom = new TalonFX(Shooter.Ports.BOTTOM_SHOOTER_MOTOR_PORT);
    acceleratorMotor = new TalonFX(Shooter.Ports.ACCELERATOR_MOTOR_PORT);
    noteSensor = new DigitalInput(Shooter.Ports.BEAM_BREAK_SENSOR_PORT);

    wristMotor.setPosition(0);
    wristMotor.clearStickyFaults();
    wristMotor.set(0);

    rollerMotorTop.clearStickyFaults();
    acceleratorMotor.clearStickyFaults();
    rollerMotorBottom.clearStickyFaults();

    rollerMotorBottom.setControl(new Follower(rollerMotorTop.getDeviceID(), true));

    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    acceleratorMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorTop.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorBottom.setNeutralMode(NeutralModeValue.Brake);

    pidController = new PIDController(0.1, 0, 0);

    targetDegrees = 0;
    wristMotorPower = 0;

    WristTab.addNumber("Current Motor Position", () -> wristMotor.getPosition().getValueAsDouble());
    WristTab.addNumber("Current motor angle", this::getCurrentAngle);
    WristTab.addNumber("Motor Power", () -> wristMotorPower);
    WristTab.addBoolean("Is at target", this::isAtTargetDegrees);
    WristTab.addNumber("Error", this::getCurrentError);
    WristTab.addNumber("target", () -> targetDegrees);
    WristTab.addNumber("Error PID", pidController::getPositionError);
  }

  private double getFeedForward() {

    // get the radians of the arm
    // getAngle() returns degrees
    double theta = Math.toRadians(getCurrentAngle());
    // get a range of 0 to 1 to multiply by feedforward.
    // when in horizontal position, value should be 1
    // when in vertical up or down position, value should be 0
    double gravityCompensation = Math.cos(theta);
    // horizontalHoldOutput is the minimum power required to hold the arm up when horizontal
    double feedForward = gravityCompensation * Shooter.HORIZONTAL_HOLD_OUTPUT;

    return 0.017; // FIXME: WHY NORA???
  }

  // wrist methods
  private double getCurrentError() {
    return targetDegrees - getCurrentAngle();
  }

  private double getCurrentAngle() {
    return rotationsToDegrees(-wristMotor.getPosition().getValue());
  }

  public boolean isAtTargetDegrees() {
    return Math.abs(getCurrentError()) < 1;
  }

  private boolean isBeamBreakSensorTriggered() {
    // if is triggered return true
    return noteSensor.get();
  }

  public boolean isDone() {
    return isBeamBreakSensorTriggered() || pose.getX() > 8.4;
  }

  public boolean isReadyToShoot() {
    return inRange
        && isAtTargetDegrees()
        && isBeamBreakSensorTriggered(); // && rotationsToDegrees(getCurrentAngle())>0
    // &&rotationsToDegrees(getCurrentAngle())<90;
  }

  public void setTargetDegrees(double degrees) {
    targetDegrees = degrees;
  }

  public void calculateWristTargetDegrees(Pose2d pose, double xV, double yV) {
    this.pose = pose;
    double g = Shooter.Measurements.GRAVITY;
    targetDegrees = getCurrentAngle();
    double x = pose.getX();
    double y = pose.getY();
    for (int i = 0; i < 5; i++) {
      // sets height and distance of NOTE based on angle (which changes where the note is)
      double d =
          Math.sqrt(Math.pow((x - Shooter.Measurements.SPEAKER_X), 2) + Math.pow((y - Shooter.Measurements.SPEAKER_Y), 2))
              - Shooter.Measurements.PIVOT_TO_ROBO_CENTER_LENGTH
              + Shooter.Measurements.NOTE_OFFSET_FROM_PIVOT_CENTER * Math.cos(targetDegrees)
              - Shooter.Measurements.PIVOT_TO_ENTRANCE_OFFSET * Math.sin(targetDegrees);
      double h =
          Shooter.Measurements.SPEAKER_HEIGHT
              - (Shooter.Measurements.PIVOT_TO_ROBO_CENTER_HEIGHT
                  + Shooter.Measurements.NOTE_OFFSET_FROM_PIVOT_CENTER * Math.sin(targetDegrees)
                  + Shooter.Measurements.PIVOT_TO_ENTRANCE_OFFSET * Math.cos(targetDegrees));

      // difference between distance to speaker now and after 1 second to find v to speaker
      double velocityToSpeaker =
          Math.sqrt((Math.pow((x - Shooter.Measurements.SPEAKER_X), 2) + Math.pow((y - Shooter.Measurements.SPEAKER_Y), 2)))
              - Math.sqrt(
                  (Math.pow((x + xV - Shooter.Measurements.SPEAKER_X), 2)
                      + Math.pow((y + yV - Shooter.Measurements.SPEAKER_Y), 2)));
      System.out.println(velocityToSpeaker);
      double v = Shooter.Measurements.NOTE_SPEED + velocityToSpeaker;

      double interiorMath = (v * v * v * v) - g * ((g * d * d) + (2 * h * v * v));
      if (interiorMath > 0) {
        targetDegrees = 180 / Math.PI * (Math.atan(((v * v) - Math.sqrt(interiorMath)) / (g * d)));
        inRange = true;
      } else {
        inRange = false;
      }
    }
  }

  private static double degreesToRotations(double angle) {
    return (angle / 360) * (Shooter.Measurements.WRIST_GEAR_RATIO);
  }

  private static double rotationsToDegrees(double rotations) {
    return (rotations / (Shooter.Measurements.WRIST_GEAR_RATIO)) * 360;
  }

  public void manualShoot() {
    rollerMotorTop.set(Shooter.ROLLER_MOTOR_POWER);
    acceleratorMotor.set(Shooter.ACCELERATOR_MOTOR_POWER);
  }

  public void manualHold() {
    rollerMotorTop.set(0);
    acceleratorMotor.set(0);
  }

  @Override
  public void periodic() {
    wristMotorPower = pidController.calculate(getCurrentAngle(), targetDegrees);
    wristMotor.set(
        -MathUtil.clamp(
            wristMotorPower + getFeedForward(),
            -0.09,
            0.09)); // you always need to incorperate feed foreward

    // if (isReadyToShoot()) {
    //   rollerMotorTop.set(Shooter.ROLLER_MOTOR_POWER);
    //   acceleratorMotor.set(Shooter.ACCELERATOR_MOTOR_POWER);
    // }
  }
}
