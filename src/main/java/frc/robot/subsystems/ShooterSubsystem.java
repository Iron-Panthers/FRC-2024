// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
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
  private double currentTime;
  private double startTime;
  private double mathJunk;
  private final ShuffleboardTab ShooterTab = Shuffleboard.getTab("Wrist");

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    wristMotor = new TalonFX(Shooter.WRIST_MOTOR_PORT);
    // rollerMotorTop = new TalonFX(Shooter.SHOOTER_MOTOR_PORT);
    // rollerMotorBottom = new TalonFX(Shooter.SHOOTER_MOTOR_PORT);
    wristMotor.clearStickyFaults();
    // rollerMotorTop.clearStickyFaults();
    // rollerMotorBottom.clearStickyFaults();
    // rollerMotorBottom.setControl(new Follower(rollerMotorTop.getDeviceID(), true));
    // rollerMotorBottom.setInverted(true);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    // rollerMotorTop.setNeutralMode(NeutralModeValue.Brake);
    // rollerMotorBottom.setNeutralMode(NeutralModeValue.Brake);
    pidController = new PIDController(0.1, 0, 0);
    startTime = 0;
    // WristTab.addNumber("Current Motor Position", wristMotor::getSelectedSensorPosition);
    // WristTab.addNumber("Current motor angle for wrist", this::getCurrentAngle);
    // WristTab.addBoolean("Is at target", this::atTargetDegrees);
    targetDegrees = 0;
    wristMotorPower =0;
    //ShooterTab.addNumber("Target Degrees10982", () -> this.targetDegrees);
    //ShooterTab.addNumber("Motor SPeed9872", () -> wristMotorPower);
  }

  public double getFeedForward() {

    // get the radians of the arm
    // getAngle() returns degrees
    double theta = Math.toRadians(getCurrentAngle());
    // get a range of 0 to 1 to multiply by feedforward.
    // when in horizontal position, value should be 1
    // when in vertical up or down position, value should be 0
    double gravityCompensation = Math.cos(theta);
    // horizontalHoldOutput is the minimum power required to hold the arm up when horizontal
    // this is a range of 0-1, in our case it was .125 throttle required to keep the arm up
    double feedForward = gravityCompensation * Shooter.HORIZONTAL_HOLD_OUTPUT;

    return 0.017;
  }
  // wrist methods
  // public void setTargetDegrees(double targetDegrees){
  //   this.targetDegrees = targetDegrees;
  //   pidController.setSetpoint(targetDegrees);
  // }
  private static double degreesToTicks(double angle) {
    return (angle * 360d) / (Shooter.WRIST_GEAR_RATIO) / (Shooter.TICKS);
  }

  private static double ticksToDegrees(double ticks) {
    return ((ticks / Shooter.TICKS / (Shooter.WRIST_GEAR_RATIO) * 360));
  }

  private double getCurrentAngle() {
    return ticksToDegrees(wristMotor.getPosition().getValue());
  }

  public void setTargetDegrees() {
    mathJunk =
        Math.pow(Shooter.NOTE_SPEED, 4)
            - Shooter.GRAVITY
                * ((Shooter.GRAVITY * Math.pow(Shooter.X_DISTANCE, 2))
                    + 2 * (Math.pow(Shooter.NOTE_SPEED, 2)) * Shooter.SPEAKER_HEIGHT);
    if (!(mathJunk > 1)) {
      mathJunk = mathJunk * -1;
    }
    targetDegrees =
        (180 / Math.PI)*(Math.atan((Shooter.NOTE_SPEED + mathJunk) / (Shooter.GRAVITY * Shooter.X_DISTANCE)));
    pidController.setSetpoint(targetDegrees);
  }
  // other methods
  // public double getCurrentTime() {
  //   return Timer.getFPGATimestamp();
  // }

  public boolean atTargetDegrees() {
    if (Math.abs(getCurrentAngle() - targetDegrees) < 1) {
      return true;
    }
    return false;
  }

  /*public boolean isDone() {

     if (getCurrentTime() - startTime > 10000) {
       return true;
     }
     return false;
   }
  */
  @Override
  public void periodic() {
    if (!atTargetDegrees()) {
      wristMotorPower = pidController.calculate(getCurrentAngle());

      wristMotor.set(wristMotorPower);
    }
    // else{
    //   if (startTime == 0){
    //     startTime = getCurrentTime();
    //   }
    //   rollerMotorTop.set(Shooter.ROLLER_MOTOR_POWER);
    // }

    // This method will be called once per scheduler run

  }
}
