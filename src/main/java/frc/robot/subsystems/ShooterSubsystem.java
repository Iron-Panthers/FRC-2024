// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import java.util.Optional;

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
  private final CANcoder wristCANcoder = new CANcoder(Shooter.Ports.CANCODER_PORT);
  private final ShuffleboardTab WristTab = Shuffleboard.getTab("Wrist");

  public ShooterSubsystem() {
    wristMotor = new TalonFX(Shooter.Ports.WRIST_MOTOR_PORT);
    rollerMotorTop = new TalonFX(Shooter.Ports.TOP_SHOOTER_MOTOR_PORT);
    //rollerMotorTop.getConfigurator().apply(new TalonFXConfiguration());
    rollerMotorBottom = new TalonFX(Shooter.Ports.BOTTOM_SHOOTER_MOTOR_PORT);
    //rollerMotorBottom.getConfigurator().apply(new TalonFXConfiguration());
    acceleratorMotor = new TalonFX(Shooter.Ports.ACCELERATOR_MOTOR_PORT);
    //acceleratorMotor.getConfigurator().apply(new TalonFXConfiguration());

    noteSensor = new DigitalInput(Shooter.Ports.BEAM_BREAK_SENSOR_PORT);

    //CAN CONFIG
    CANcoderConfiguration wristCANcoderConfig = new CANcoderConfiguration();
    wristCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    wristCANcoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue
            .Clockwise_Positive; // counter clockwise is default, false is counter clockwise
    wristCANcoderConfig.MagnetSensor.MagnetOffset = Shooter.Measurements.WRIST_CANCODER_OFFSET;
    wristCANcoder.getConfigurator().apply(wristCANcoderConfig);

    //WRIST MOROT CONFIG
    TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
    wristMotorConfig.Feedback.FeedbackRemoteSensorID = wristCANcoder.getDeviceID();
    wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    wristMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    wristMotorConfig.Feedback.RotorToSensorRatio = Shooter.Measurements.WRIST_GEAR_RATIO;
    wristMotorConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(-0.27);
    wristMotorConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-0.434);
    wristMotorConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    wristMotorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    wristMotorConfig.Voltage.withPeakForwardVoltage(1);
    wristMotorConfig.Voltage.withPeakReverseVoltage(-1);

    wristMotor.getConfigurator().apply(wristMotorConfig);

    
    // wristMotor.setPosition(0);
    wristMotor.clearStickyFaults();
    wristMotor.setInverted(true);

    rollerMotorTop.clearStickyFaults();
    acceleratorMotor.clearStickyFaults();
    rollerMotorBottom.clearStickyFaults();

    rollerMotorBottom.setControl(new Follower(rollerMotorTop.getDeviceID(), true));

    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    acceleratorMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorTop.setNeutralMode(NeutralModeValue.Brake);
    rollerMotorBottom.setNeutralMode(NeutralModeValue.Brake);

    pidController = new PIDController(80, 0, 0);

    targetDegrees = 0;
    wristMotorPower = 0;

    WristTab.addNumber("Current Motor Position", () -> wristMotor.getPosition().getValueAsDouble());
    WristTab.addNumber("Current motor angle", this::getCurrentAngle);
    WristTab.addNumber("Motor Power", () -> wristMotorPower);
    WristTab.addBoolean("Is at target", this::isAtTargetDegrees);
    WristTab.addNumber("Error", this::getCurrentError);
    WristTab.addNumber("target", this::getTargetDegrees);
    WristTab.addNumber("Error PID", pidController::getPositionError);
    WristTab.addNumber("Applied Voltage", () -> wristMotor.getMotorVoltage().getValueAsDouble());
    WristTab.add(pidController);
  }

  // wrist methods
  private double getCurrentError() {
    return targetDegrees - getCurrentAngle();
  }

  private double getCurrentAngle() {
    return rotationsToDegrees(wristMotor.getPosition().getValue());
  }

  private double getTargetDegrees() {
    return targetDegrees;
  }

  public boolean isAtTargetDegrees() {
    return Math.abs(getCurrentError()) < 1;
  }

  public boolean isShooterUpToSpeed() {
    return rollerMotorBottom.getVelocity().getValueAsDouble() > 50
        && rollerMotorTop.getVelocity().getValueAsDouble() > 50;
  }

  private boolean isBeamBreakSensorTriggered() {
    // if is triggered return true
    return noteSensor.get();
  }

  public boolean isDone() {
    return isBeamBreakSensorTriggered() || pose.getX() > 8.4;
  }

  public boolean prepareForIntake() {
    if (getCurrentAngle() > 20) {
      setTargetDegrees(20);
      return false;
    }
    return true;
  }

  public boolean isReadyToShoot() {
    return inRange
        && isAtTargetDegrees()
        && isBeamBreakSensorTriggered(); // && rotationsToDegrees(getCurrentAngle())>0
    // &&rotationsToDegrees(getCurrentAngle())<90;
  }

  public void setTargetDegrees(double degrees) {
    this.targetDegrees = degrees;
  }

  public void startShooterMotor() {
    rollerMotorTop.set(Shooter.ROLLER_MOTOR_POWER);
  }

  public void setAcceleratorMotorSpeed(double speed) {
    acceleratorMotor.set(speed);
  }

  public void calculateWristTargetDegrees(Pose2d pose, double xV, double yV) {
    this.pose = pose;
    double g = Shooter.Measurements.GRAVITY;
    double x = pose.getX();
    double y = pose.getY();
    double speakerX;
    double speakerY;
    targetDegrees = getCurrentAngle();
    Optional<Alliance> color = DriverStation.getAlliance();

    if (color.isPresent() && color.get() == Alliance.Blue) {
      speakerX = Shooter.Measurements.BLUE_SPEAKER_POSE.getX();
      speakerY = Shooter.Measurements.BLUE_SPEAKER_POSE.getY();
    } else {
      speakerX = Shooter.Measurements.RED_SPEAKER_POSE.getX();
      speakerY = Shooter.Measurements.RED_SPEAKER_POSE.getY();
    }
    double distanceToSpeaker = Math.sqrt(Math.pow((x - speakerX), 2) + Math.pow((y - speakerY), 2));

    for (int i = 0; i < 5; i++) {
      // Finds the height and distance of NOTE from the speaker based on angle (which changes where
      // the note is)

      double d =
          distanceToSpeaker
              - Shooter.Measurements.PIVOT_TO_ROBO_CENTER_LENGTH
              + Shooter.Measurements.NOTE_OFFSET_FROM_PIVOT_CENTER * Math.cos(targetDegrees)
              - Shooter.Measurements.PIVOT_TO_ENTRANCE_OFFSET
                  * Math.sin(targetDegrees); // FIXME maybe change to cos()?

      double h =
          Shooter.Measurements.SPEAKER_HEIGHT
              - (Shooter.Measurements.PIVOT_TO_ROBO_CENTER_HEIGHT
                  + Shooter.Measurements.NOTE_OFFSET_FROM_PIVOT_CENTER * Math.sin(targetDegrees)
                  + Shooter.Measurements.PIVOT_TO_ENTRANCE_OFFSET
                      * Math.cos(targetDegrees)); // FIXME maybe change to sin() for height?

      // difference between distance to speaker now and after 1 second to find v to speaker
      double velocityToSpeaker =
          distanceToSpeaker
              - Math.sqrt((Math.pow((x + xV - speakerX), 2) + Math.pow((y + yV - speakerY), 2)));

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
    return (angle / 360);
  }

  private static double rotationsToDegrees(double rotations) {
    return (rotations * 360);
  }

  @Override
  public void periodic() {
    wristMotorPower = pidController.calculate(getCurrentAngle(), targetDegrees);

    wristMotor.set(MathUtil.clamp(
            wristMotorPower + Shooter.HORIZONTAL_HOLD_OUTPUT,
            -0.09,
            0.09)); // you always need to incorperate feed foreward


  }
}
