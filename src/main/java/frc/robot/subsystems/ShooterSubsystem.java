// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
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

  // DEBUG
  private double d_topToBottomRatio = .068d;
  private GenericEntry ampRollerRatioEntry;

  private double d_ShooterSpeed = .5d;
  private GenericEntry shooterSpeedEntry;
  private GenericEntry useDebugControls;

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0).withSlot(0);

  private final ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");

  public enum ShooterMode {
    INTAKE(Shooter.Modes.INTAKE),
    IDLE(Shooter.Modes.IDLE),
    RAMP_SPEAKER(Shooter.Modes.RAMP_SPEAKER),
    RAMP_AMP_BACK(Shooter.Modes.RAMP_AMP_BACK),
    RAMP_AMP_FRONT(Shooter.Modes.RAMP_AMP_FRONT),
    SHOOT_SPEAKER(Shooter.Modes.SHOOT_SPEAKER),
    SHOOT_AMP_BACK(Shooter.Modes.SHOOT_AMP_BACK),
    SHOOT_AMP_FORWARD(Shooter.Modes.SHOOT_AMP_FORWARD);

    public final ShooterPowers shooterPowers;

    private ShooterMode(ShooterPowers shooterPowers) {
      this.shooterPowers = shooterPowers;
    }
  }

  public record ShooterPowers(double roller, double topToBottomRatio, double accelerator) {
    public ShooterPowers(double roller, double topToBottomRatio, double accelerator) {
      this.roller = roller;
      this.topToBottomRatio = topToBottomRatio;
      this.accelerator = accelerator;
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

    rollerMotorBottom.setControl(new Follower(rollerMotorTop.getDeviceID(), false));

    acceleratorMotor.setInverted(true);
    rollerMotorBottom.setInverted(true);
    rollerMotorTop.setInverted(true);

    acceleratorMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorTop.setNeutralMode(NeutralModeValue.Coast);
    rollerMotorBottom.setNeutralMode(NeutralModeValue.Coast);

    shooterMode = ShooterMode.IDLE;

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
      shooterTab.addString("mode", () -> shooterMode.toString());

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

  public ShooterMode getMode() {
    return shooterMode;
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

  public void advanceToShootMode() {
    switch (shooterMode) {
      case RAMP_AMP_FRONT:
        shooterMode = ShooterMode.SHOOT_AMP_FORWARD;
        break;
      case RAMP_AMP_BACK:
        shooterMode = ShooterMode.SHOOT_AMP_BACK;
        break;
      case RAMP_SPEAKER:
      default:
        shooterMode = ShooterMode.SHOOT_SPEAKER;
    }
  }

  @Override
  public void periodic() {
    if (useDebugControls.getBoolean(false)) {
      // In debug use all of the debug values for our mode
      d_topToBottomRatio = ampRollerRatioEntry.getDouble(1);
      d_ShooterSpeed = shooterSpeedEntry.getDouble(0);
      rollerMotorBottom.setControl(velocityVoltageRequest.withVelocity(d_ShooterSpeed));
      rollerMotorTop.setControl(
          velocityVoltageRequest.withVelocity(d_ShooterSpeed * d_topToBottomRatio));
    } else {
      // Otherwise just use our current mode for the values
      rollerMotorBottom.setControl(
          velocityVoltageRequest.withVelocity(shooterMode.shooterPowers.roller()));
      rollerMotorTop.setControl(
          velocityVoltageRequest.withVelocity(
              shooterMode.shooterPowers.roller() * shooterMode.shooterPowers.topToBottomRatio()));
    }

    acceleratorMotor.set(shooterMode.shooterPowers.accelerator());
  }
}
