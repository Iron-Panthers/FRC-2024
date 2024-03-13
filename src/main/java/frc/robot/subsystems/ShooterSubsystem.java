// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;
import frc.robot.Constants.Shooter;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX rollerMotorBottom;
  private final TalonFX rollerMotorTop;
  private final TalonFX acceleratorMotor;

  private DigitalInput noteSensor;

  private ShooterMode shooterMode;

  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

  // VELOCITY PID
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0).withSlot(0);

  // DEBUG
  private double d_AmpRollerRatio = .068d;
  private GenericEntry ampRollerRatioEntry;

  private double d_ShooterSpeed = .5d;
  private GenericEntry shooterSpeedEntry;
  private GenericEntry useDebugControls;

  public enum ShooterMode {
    INTAKE(Shooter.Modes.INTAKE),
    IDLE(Shooter.Modes.IDLE),
    RAMPING_SPEAKER(Shooter.Modes.RAMP_SPEAKER),
    RAMPING_AMP(Shooter.Modes.RAMP_AMP),
    SHOOTING_SPEAKER(Shooter.Modes.SHOOT_SPEAKER),
    SHOOTING_AMP(Shooter.Modes.SHOOT_AMP);

    public final ShooterSpeeds shooterSpeeds;

    private ShooterMode(ShooterSpeeds shooterPowers) {
      this.shooterSpeeds = shooterPowers;
    }
  }

  public record ShooterSpeeds(double roller, double accelerator, double rollerRatio) {
    /**
     * Custom motor powers for each shooter mode.
     *
     * @param roller rps of roller motor
     * @param accelerator rps of acceletator motor
     * @param rollerRatio the ratio between the top and bottom roller: top speed = bottom *
     *     rollerRatio
     */
    public ShooterSpeeds(double roller, double accelerator, double rollerRatio) {
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

    var rollerConfigs = new Slot0Configs();
    rollerConfigs.kS = 0.23; // Add 0.05 V output to overcome static friction
    rollerConfigs.kV = 0.24; // A velocity target of 1 rps results in 0.12 V output
    rollerConfigs.kP = 0.2; // An error of 1 rps results in 0.11 V output
    rollerConfigs.kI = 0; // no output for integrated error
    rollerConfigs.kD = 0; // no output for error derivative

    rollerMotorBottom.getConfigurator().apply(rollerConfigs);
    rollerMotorTop.getConfigurator().apply(rollerConfigs);

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      shooterTab.addBoolean("Sensor Input", this::isBeamBreakSensorTriggered);
      shooterTab
          .addDouble(
              "Top Roller Velocity (RPS)", () -> rollerMotorTop.getVelocity().getValueAsDouble())
          .withWidget(BuiltInWidgets.kGraph)
          .withSize(2, 1);
      shooterTab
          .addDouble(
              "Bottom Roller Velocity (RPS)",
              () -> rollerMotorBottom.getVelocity().getValueAsDouble())
          .withWidget(BuiltInWidgets.kGraph)
          .withSize(2, 1);
      shooterTab.addDouble(
          "Top roller amps", () -> rollerMotorTop.getSupplyCurrent().getValueAsDouble());
      shooterTab.addDouble(
          "Bottom roller amps", () -> rollerMotorBottom.getSupplyCurrent().getValueAsDouble());

      ampRollerRatioEntry =
          shooterTab
              .add("DEBUG Amp Top to Bottom Roller Ratio", 1)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0, "max", 1))
              .withSize(3, 1)
              .getEntry();
      shooterSpeedEntry =
          shooterTab
              .add("DEBUG Shooter Velocity", .5)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0, "max", 90))
              .withSize(3, 1)
              .getEntry();
      useDebugControls =
          shooterTab
              .add("Use Debug Controls", false)
              .withWidget(BuiltInWidgets.kToggleSwitch)
              .withSize(2, 1)
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

  public ShooterMode getMode() {
    return shooterMode;
  }

  public void haltAccelerator() {
    acceleratorMotor.set(0);
  }

  public void setShooterMode(ShooterMode shooterMode) {
    this.shooterMode = shooterMode;
  }

  @Override
  public void periodic() {
    if (useDebugControls.getBoolean(false)) {
      // In debug use all of the debug values for our mode
      d_AmpRollerRatio = ampRollerRatioEntry.getDouble(1);
      d_ShooterSpeed = shooterSpeedEntry.getDouble(0);
      rollerMotorBottom.setControl(velocityVoltageRequest.withVelocity(d_ShooterSpeed));
      rollerMotorTop.setControl(
          velocityVoltageRequest.withVelocity(d_ShooterSpeed * d_AmpRollerRatio));
    } else {
      // Otherwise just use our current mode for the values
      rollerMotorBottom.setControl(
          velocityVoltageRequest.withVelocity(shooterMode.shooterSpeeds.roller()));
      rollerMotorTop.setControl(
          velocityVoltageRequest.withVelocity(
              shooterMode.shooterSpeeds.roller() * shooterMode.shooterSpeeds.rollerRatio()));
    }

    acceleratorMotor.set(shooterMode.shooterSpeeds.accelerator());
  }
}
