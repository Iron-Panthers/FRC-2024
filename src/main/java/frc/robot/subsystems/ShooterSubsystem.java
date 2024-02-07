// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX wristMotor;
  private TalonFX rollerMotorBottom;
  private TalonFX rollerMotorTop;
  private PIDController pidController;
  private double targetDegrees;
  private double wristMotorPower;
  private Pose2d pose;
  private double pastShooterHeight;
  private double pastCenterOfNote;
  private final ShuffleboardTab WristTab = Shuffleboard.getTab("Wrist");

  public ShooterSubsystem() {
    wristMotor = new TalonFX(Shooter.WRIST_MOTOR_PORT);
    rollerMotorTop = new TalonFX(Shooter.SHOOTER_MOTOR_PORT);
    rollerMotorBottom = new TalonFX(Shooter.SHOOTER_MOTOR_PORT);
    this.wristMotor.setPosition(0);
    wristMotor.clearStickyFaults();
    this.wristMotor.set(0);
    rollerMotorTop.clearStickyFaults();
    rollerMotorBottom.clearStickyFaults();
    rollerMotorBottom.setControl(new Follower(rollerMotorTop.getDeviceID(), true));
    rollerMotorBottom.setInverted(true);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorTop.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorBottom.setNeutralMode(NeutralModeValue.Brake);
    pidController = new PIDController(0.1, 0, 0);
    WristTab.addNumber("Current Motor Position", () -> wristMotor.getPosition().getValueAsDouble());
    WristTab.addNumber("Current motor angle", this::getCurrentAngle);
    WristTab.addNumber("Motor Power", () -> wristMotorPower);
    WristTab.addBoolean("Is at target", this::isAtTargetDegrees);
    WristTab.addNumber("Error", this::getCurrentError);
    WristTab.addNumber("target", () -> targetDegrees);
    WristTab.addNumber("Error PID", pidController::getPositionError);
    targetDegrees = 0;
    wristMotorPower = 0;
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

    return 0.017;
  }

  // wrist methods
  private double getCurrentError() {
    return targetDegrees - getCurrentAngle();
  }

  private double getCurrentAngle() {
    return -rotationsToDegrees(wristMotor.getPosition().getValue());
  }

  public boolean isAtTargetDegrees() {
    return getCurrentError() < 1;
  }

  private boolean isSensorTriggered() {
    // if is triggered return true
    return false;
  }

  public boolean isDone() {
    return isSensorTriggered() || pose.getX() > 8.4;
  }




  public boolean isReadyToShoot() {
    return pose.getX() < 8.4 && isAtTargetDegrees();
  }

  public void calculateWristTargetDegrees(Pose2d pose, double xV, double yV) {
    this.pose = pose;
    double g = Shooter.GRAVITY;
    targetDegrees = getCurrentAngle();
    double x = pose.getX();
    double y = pose.getY();
    for (int i = 0; i<5; i++){
        //sets height and distance of NOTE based on angle (which changes where the note is)
    double d = Math.sqrt(Math.pow((x - Shooter.SPEAKER_X), 2)
      + Math.pow((y - Shooter.SPEAKER_Y), 2)) - Shooter.PIVOT_TO_ROBO_CENTER_LENGTH + Shooter.NOTE_OFFSET_FROM_PIVOT_CENTER*Math.cos(targetDegrees) - Shooter.PIVOT_TO_ENTRANCE_OFFSET * Math.sin(targetDegrees);
    double h = Shooter.SPEAKER_HEIGHT - (Shooter.PIVOT_TO_ROBO_CENTER_HEIGHT + Shooter.NOTE_OFFSET_FROM_PIVOT_CENTER * Math.sin(targetDegrees) + Shooter.PIVOT_TO_ENTRANCE_OFFSET * Math.cos(targetDegrees));

    // difference between distance to speaker now and after 1 second to find v to speaker
    double velocityToSpeaker =
    Math.sqrt((Math.pow((x - Shooter.SPEAKER_X), 2)
    + Math.pow((y - Shooter.SPEAKER_Y), 2)))
    - Math.sqrt((Math.pow((x + xV - Shooter.SPEAKER_X), 2)
    + Math.pow((y + yV - Shooter.SPEAKER_Y), 2)));
System.out.println(velocityToSpeaker);
    double v = Shooter.NOTE_SPEED + velocityToSpeaker;
    

    double interiorMath = (v*v*v*v)-g*((g*d*d)+(2*h*v*v));
    if (interiorMath>0){
      targetDegrees = 180/Math.PI*(Math.atan(((v*v)-Math.sqrt(interiorMath))/(g*d)));
    }
    else{
      targetDegrees = 0;
    }

  }

    pidController.setSetpoint(targetDegrees);
  }
  
  private static double degreesToTicks(double angle) {
    return (angle * 360) / (Shooter.WRIST_GEAR_RATIO);
  }

  private static double rotationsToDegrees(double rotations) {
    return ((rotations / (Shooter.WRIST_GEAR_RATIO) * 360));
  }

  @Override
  public void periodic() {
    wristMotorPower = pidController.calculate(getCurrentAngle());
    if (!isReadyToShoot()) {
      wristMotor.set(
          -MathUtil.clamp(
              wristMotorPower + getFeedForward(),
              -0.5,
              0.5)); // you allways need to incorperate feed foreward
    }
    if (isReadyToShoot()) {
      wristMotor.set(-MathUtil.clamp(wristMotorPower + getFeedForward(), -0.1, 0.1));
      rollerMotorTop.set(Shooter.ROLLER_MOTOR_POWER);
    }
  }
}
