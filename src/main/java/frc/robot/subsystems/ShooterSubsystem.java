// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  //DEBUG
  private double d_AmpRollerRatio = .068d;
  private GenericEntry ampRollerRatioEntry;

  private double d_ShooterSpeed = .5d;
  private GenericEntry shooterSpeedEntry;


  public enum ShooterMode {
    INTAKE(Shooter.Modes.INTAKE),
    IDLE(Shooter.Modes.IDLE),
    RAMPING(Shooter.Modes.RAMPING),
    SHOOTING_SPEAKER(Shooter.Modes.SHOOT_SPEAKER),
    SHOOTING_AMP(Shooter.Modes.SHOOT_AMP);


    public final ShooterPowers shooterPowers;

    private ShooterMode(ShooterPowers shooterPowers) {
      this.shooterPowers = shooterPowers;
    }
  }

  public record ShooterPowers(double roller, double accelerator, double rollerRatio) {
    /**
     * Custom motor powers for each shooter mode.
     * @param roller percent speed of roller motor
     * @param accelerator percent speed of acceletator motor
     * @param rollerRatio the ratio between the top and bottom roller: top speed = bottom * rollerRatio
     */
    public ShooterPowers(double roller, double accelerator, double rollerRatio) {
      this.roller = roller;
      this.accelerator = accelerator;
      this.rollerRatio = rollerRatio;
    }
  }

  public ShooterSubsystem() {
    rollerMotorTop = new TalonFX(Shooter.Ports.TOP_SHOOTER_MOTOR_PORT);
    rollerMotorBottom = new TalonFX(Shooter.Ports.BOTTOM_SHOOTER_MOTOR_PORT);
    acceleratorMotor = new TalonFX(Shooter.Ports.ACCELERATOR_MOTOR_PORT);

    noteSensor = new DigitalInput(Shooter.Ports.BEAM_BREAK_SENSOR_PORT);

    // rollerMotorTop.getConfigurator().apply(new TalonFXConfiguration());
    // rollerMotorBottom.getConfigurator().apply(new TalonFXConfiguration());
    // acceleratorMotor.getConfigurator().apply(new TalonFXConfiguration());

    rollerMotorTop.clearStickyFaults();
    acceleratorMotor.clearStickyFaults();
    rollerMotorBottom.clearStickyFaults();

    acceleratorMotor.setInverted(true);
    rollerMotorBottom.setInverted(true);
    rollerMotorTop.setInverted(true);

    acceleratorMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorTop.setNeutralMode(NeutralModeValue.Coast);
    rollerMotorBottom.setNeutralMode(NeutralModeValue.Coast);

    shooterMode = ShooterMode.IDLE;

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      shooterTab.addBoolean("Sensor Input", this::isBeamBreakSensorTriggered);
      shooterTab.addDouble(
          "Top Roller Velocity", () -> rollerMotorTop.getVelocity().getValueAsDouble());
      shooterTab.addDouble(
          "Bottom Roller Velocity", () -> rollerMotorBottom.getVelocity().getValueAsDouble());
      shooterTab.addDouble(
          "Top roller amps", () -> rollerMotorTop.getSupplyCurrent().getValueAsDouble());
      shooterTab.addDouble(
          "Bottom roller amps", () -> rollerMotorBottom.getSupplyCurrent().getValueAsDouble());

      ampRollerRatioEntry = shooterTab.add(
          "DEBUG Amp Top Roller Percent", 1)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1))
          .getEntry();
      shooterSpeedEntry = shooterTab.add(
          "DEBUG Shooter Speed", .5)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1))
          .getEntry();
    }
  }

  public boolean isShooterUpToSpeed() {
    return rollerMotorBottom.getVelocity().getValueAsDouble() >= Shooter.SHOOTER_VELOCITY_THRESHOLD
        && rollerMotorTop.getVelocity().getValueAsDouble() >= Shooter.SHOOTER_VELOCITY_THRESHOLD;
  }

  public boolean isBeamBreakSensorTriggered() {
    return !noteSensor.get();
  }

  public boolean isReadyToShoot() {
    return isBeamBreakSensorTriggered() && isShooterUpToSpeed();
  }

  public void haltAccelerator() {
    acceleratorMotor.set(0);
  }

  public void setShooterMode(ShooterMode shooterMode) {
    this.shooterMode = shooterMode;
  }

  @Override
  public void periodic() {
    d_AmpRollerRatio = ampRollerRatioEntry.getDouble(1);
    d_ShooterSpeed = shooterSpeedEntry.getDouble(.5);

    rollerMotorBottom.set(d_ShooterSpeed);
    rollerMotorTop.set(d_ShooterSpeed * d_AmpRollerRatio);// * shooterMode.shooterPowers.rollerRatio());

    acceleratorMotor.set(shooterMode.shooterPowers.accelerator());
  }
}
