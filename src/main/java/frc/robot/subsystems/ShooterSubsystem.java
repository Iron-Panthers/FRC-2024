// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX rollerMotorBottom;
  private final TalonFX rollerMotorTop;
  private final TalonFX acceleratorMotor;

  private DigitalInput noteSensor;

  private ShooterMode shooterMode;

  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

  public enum ShooterMode {
    INTAKE(Shooter.Modes.INTAKE),
    IDLE(Shooter.Modes.IDLE),
    RAMPING(Shooter.Modes.RAMPING),
    SHOOTING(Shooter.Modes.SHOOTING);

    public final ShooterPowers shooterPowers;

    private ShooterMode(ShooterPowers shooterPowers) {
      this.shooterPowers = shooterPowers;
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
    rollerMotorTop = new TalonFX(Shooter.Ports.TOP_SHOOTER_MOTOR_PORT);
    // rollerMotorTop.getConfigurator().apply(new TalonFXConfiguration());
    rollerMotorBottom = new TalonFX(Shooter.Ports.BOTTOM_SHOOTER_MOTOR_PORT);
    // rollerMotorBottom.getConfigurator().apply(new TalonFXConfiguration());
    acceleratorMotor = new TalonFX(Shooter.Ports.ACCELERATOR_MOTOR_PORT);
    // acceleratorMotor.getConfigurator().apply(new TalonFXConfiguration());
    noteSensor = new DigitalInput(Shooter.Ports.BEAM_BREAK_SENSOR_PORT);

    rollerMotorTop.clearStickyFaults();
    acceleratorMotor.clearStickyFaults();
    rollerMotorBottom.clearStickyFaults();

    rollerMotorBottom.setControl(new Follower(rollerMotorTop.getDeviceID(), false));

    acceleratorMotor.setInverted(true);
    rollerMotorBottom.setInverted(true);
    rollerMotorTop.setInverted(true);
    
    acceleratorMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorTop.setNeutralMode(NeutralModeValue.Coast);
    rollerMotorBottom.setNeutralMode(NeutralModeValue.Coast);

    shooterMode = ShooterMode.IDLE;

    // SHUFFLEBOARD
      shooterTab.addBoolean("Sensor Input", this::isBeamBreakSensorTriggered);
      shooterTab.addDouble(
          "Top Roller Velocity", () -> rollerMotorTop.getVelocity().getValueAsDouble());
      shooterTab.addDouble(
          "Bottom Roller Velocity", () -> rollerMotorBottom.getVelocity().getValueAsDouble());
      shooterTab.addDouble("Top roller amps", () -> rollerMotorTop.getSupplyCurrent().getValueAsDouble());
      shooterTab.addDouble("Bottom roller amps", () -> rollerMotorBottom.getSupplyCurrent().getValueAsDouble());
  }

  public boolean isShooterUpToSpeed() {
    return rollerMotorBottom.getVelocity().getValueAsDouble() >= Shooter.REQUIRED_SHOOT_SPEED
        && rollerMotorTop.getVelocity().getValueAsDouble() >= Shooter.REQUIRED_SHOOT_SPEED;
  }

  public boolean isBeamBreakSensorTriggered() {
    // if is triggered return true
    return !noteSensor.get();
  }

  public boolean isReadyToShoot() {
    return isBeamBreakSensorTriggered() && isShooterUpToSpeed();
  }

  public void stopAccelerator() {
    acceleratorMotor.set(0);
  }

  public void setShooterMode(ShooterMode newMode) {
    this.shooterMode = newMode;
  }

  @Override
  public void periodic() {
    // shooter motor power
    rollerMotorTop.set(shooterMode.shooterPowers.roller());
    acceleratorMotor.set(shooterMode.shooterPowers.accelerator());
  }
}
