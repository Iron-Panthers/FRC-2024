// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX wristMotor;
  private TalonFX rollerMotor;
  private PIDController pidController;
  private double targetDegrees;
  private double wristMotorPower;
  private double rollerMotorPower;
  private double currentTime;
  private double startTime;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    wristMotor = new TalonFX(Shooter.WRIST_MOTOR_PORT); 
    rollerMotor = new TalonFX(Shooter.SHOOTER_MOTOR_PORT);
    wristMotor.clearStickyFaults();
    rollerMotor.clearStickyFaults();
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    pidController = new PIDController(0.1, 0, 0);
    startTime = 0;
  }


  //wrist methods
  public void setTargetDegrees(double targetDegrees){
    this.targetDegrees = targetDegrees;
    pidController.setSetpoint(targetDegrees);
  }
  private static double degreesToTicks(double angle) {
    return (angle * 360d) / (Shooter.WRIST_GEAR_RATIO) / (Shooter.TICKS);
  }
  private static double ticksToDegrees(double ticks) {
    return ((ticks / Shooter.TICKS / (Shooter.WRIST_GEAR_RATIO) * 360));
  }
  private double getCurrentAngle() {
    return ticksToDegrees(wristMotor.getPosition().getValue());
  }
  //other methods
  public double getCurrentTime(){
        return Timer.getFPGATimestamp();
  }
  public boolean atTargetDegrees(){
    if (Math.abs(getCurrentAngle() - targetDegrees)<1) {
      return true;
    }
    return false;
  }
  public boolean isDone(){
    
    if (getCurrentTime()- startTime>3000){
          return true;
    }
    return false;

  }
  @Override
  public void periodic() {
    if (!atTargetDegrees()){
      wristMotorPower = pidController.calculate(getCurrentAngle());
    
    wristMotor.set(wristMotorPower);
    }
    else{
      if (startTime == 0){
        startTime = getCurrentTime();
      }
      rollerMotor.set(rollerMotorPower);
    }

    // This method will be called once per scheduler run

  }
}
