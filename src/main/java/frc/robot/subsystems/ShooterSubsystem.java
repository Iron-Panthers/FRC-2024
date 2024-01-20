// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX wristMotor;
  private TalonFX rollerMotor;
  private PIDController pidController;
  private double targetDegrees;
  private double wristMotorPower;
  private double rollerMotorPower;
  private ShuffleboardTab shuffleboard = Shuffleboard.getTab("Shooter Subsystem");

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    wristMotor = new TalonFX(Shooter.WRIST_MOTOR_PORT);
    rollerMotor = new TalonFX(Shooter.SHOOTER_MOTOR_PORT);
    wristMotor.clearStickyFaults();
    rollerMotor.clearStickyFaults();
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    pidController = new PIDController(0.1, 0, 0);

    shuffleboard.addDouble(
        "Wrist Motor Voltage", () -> wristMotor.getMotorVoltage().getValueAsDouble());
    shuffleboard.addDouble(
        "Roller Motor Voltage", () -> rollerMotor.getMotorVoltage().getValueAsDouble());
    shuffleboard.addDouble("Target Wrist Power", () -> wristMotorPower);
    shuffleboard.addDouble("Target Roller Power", () -> rollerMotorPower);
    shuffleboard.addDouble("Wrist Motor Angle", this::getCurrentAngle);
    shuffleboard.addDouble("Target Angle", () -> targetDegrees);
    shuffleboard.add("PID", pidController);
  }
  // wrist methods
  public void setTargetDegrees(double targetDegrees) {
    this.targetDegrees = targetDegrees;
    pidController.setSetpoint(targetDegrees);
  }

  public void setRollerPower(double rollerMotorPower) {
    this.rollerMotorPower = rollerMotorPower;
  }

  private static double degreesToTicks(double angle) {
    return (angle * 360) / (Shooter.WRIST_GEAR_RATIO) / (Shooter.TICKS);
  }

  private static double ticksToDegrees(double ticks) {
    return ((ticks / Shooter.TICKS / (Shooter.WRIST_GEAR_RATIO) * 360));
  }

  private double getCurrentAngle() {
    return ticksToDegrees(wristMotor.getPosition().getValue());
  }
  // sensor methods
  public static boolean isDone() {
    // is sensor triggered?
    return false;
  }

  @Override
  public void periodic() {
    wristMotorPower = pidController.calculate(getCurrentAngle());
    wristMotor.set(wristMotorPower);
    rollerMotor.set(rollerMotorPower);
    // This method will be called once per scheduler run
  }
}
