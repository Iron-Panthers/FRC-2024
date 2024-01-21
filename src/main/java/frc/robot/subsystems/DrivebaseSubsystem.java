// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Config;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Dims;
import frc.robot.Constants.PoseEstimator;
import frc.robot.subsystems.VisionSubsystem.VisionMeasurement;
import frc.util.Util;

public class DrivebaseSubsystem extends SubsystemBase {
  /**
   * The kinematics object allows us to encode our relationship between desired speeds (represented
   * by a ChassisSpeeds object) and our actual drive outputs (what speeds and angles we apply to
   * each module)
   */
  // FIXME check if its correct for season
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          // Front right
          new Translation2d(Dims.TRACKWIDTH_METERS / 2.0, -Dims.WHEELBASE_METERS / 2.0),
          // Front left
          new Translation2d(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-Dims.TRACKWIDTH_METERS / 2.0, -Dims.WHEELBASE_METERS / 2.0));

  /**
   * Object handles configuration and control of drivetrain. Also contains each swerve module.
   * Order: FR, FL, BL, BR. Or in Quadrants: I, II, III, IV Handles odometry, but unsure if it's
   * better to do it ourselves
   */
  private final SwerveDrivetrain swerveDrivetrain;
  private final SwerveModule[] swerveModules;

  /** The SwerveDriveOdometry class allows us to estimate the robot's "pose" over time. */
  private final SwerveDrivePoseEstimator swervePoseEstimator;

  private final VisionSubsystem visionSubsystem;

  /**
   * Keeps track of the current estimated pose (x,y,theta) of the robot, as estimated by odometry.
   */
  private Pose2d robotPose = new Pose2d();

  /** The current ChassisSpeeds goal for the drivetrain */
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); // defaults to zeros

  /* Requests to pass to SwerveDrivetrain objects */
  private ApplyChassisSpeeds chassisSpeedRequest = new ApplyChassisSpeeds().withDriveRequestType(Modules.Params.driveRequestType).withSteerRequestType(Modules.Params.steerRequestType);
  private SwerveDriveBrake swerveBrakeRequest = new SwerveDriveBrake();

  /** The modes of the drivebase subsystem */
  public enum Modes {
    DRIVE,
    DRIVE_ANGLE,
    DEFENSE
  }

  /** The current mode */
  private Modes mode = Modes.DRIVE;

  private Field2d field = new Field2d();

  private final PIDController rotController;

  private double targetAngle = 0; // default target angle to zero

  private Pair<Double, Double> xyInput = new Pair<>(0d, 0d); // the x and y for using target angles
  /**
   * The Shuffleboard tab which all things related to the drivebase can be put for easy access and
   * organization
   */
  private final ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

  /** Creates a new DrivebaseSubsystem. */
  public DrivebaseSubsystem(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;

    if (!Config.DISABLE_SWERVE_INIT) {
      final SwerveDrivetrainConstants swerveDrivetrainConstants =
          new SwerveDrivetrainConstants().withPigeon2Id(PIGEON_PORT).withCANbusName(SWERVE_CANBUS);

      final SwerveModuleConstantsFactory swerveModuleConstantsFactory =
          new SwerveModuleConstantsFactory()
              .withWheelRadius(Modules.Params.WHEEL_RADIUS)
              .withCouplingGearRatio(Modules.Params.COUPLING_GEAR_RATIO)
              .withDriveMotorGearRatio(Modules.Params.DRIVE_GEAR_RATIO)
              .withSteerMotorGearRatio(Modules.Params.STEER_GEAR_RATIO)
              .withDriveMotorGains(Modules.Params.DRIVE_MOTOR_GAINS)
              .withSteerMotorGains(Modules.Params.STEER_MOTOR_GAINS)
              .withDriveMotorClosedLoopOutput(Modules.Params.DRIVE_CLOSED_LOOP_OUTPUT)
              .withSteerMotorClosedLoopOutput(Modules.Params.STEER_CLOSED_LOOP_OUTPUT)
              .withFeedbackSource(Modules.Params.FEEDBACK_SOURCE)
              .withSpeedAt12VoltsMps(Modules.Params.SPEED_TWELVE_VOLTS);
      // .withSlipCurrent(Modules.Params.SLIP_CURRENT);

      // module wheel positions taken from kinematics object
      final SwerveModuleConstants frontLeft =
          swerveModuleConstantsFactory.createModuleConstants(
              Modules.Module4.STEER_MOTOR,
              Modules.Module4.DRIVE_MOTOR,
              Modules.Module4.STEER_ENCODER,
              Modules.Module4.STEER_OFFSET,
              Dims.TRACKWIDTH_METERS / 2.0,
              Dims.TRACKWIDTH_METERS / 2.0,
              false);

      // module wheel positions taken from kinematics object
      final SwerveModuleConstants frontRight =
          swerveModuleConstantsFactory.createModuleConstants(
              Modules.Module3.STEER_MOTOR,
              Modules.Module3.DRIVE_MOTOR,
              Modules.Module3.STEER_ENCODER,
              Modules.Module3.STEER_OFFSET,
              Dims.TRACKWIDTH_METERS / 2.0,
              -Dims.TRACKWIDTH_METERS / 2.0,
              true);

      // module wheel positions taken from kinematics object
      final SwerveModuleConstants backLeft =
          swerveModuleConstantsFactory.createModuleConstants(
              Modules.Module1.STEER_MOTOR,
              Modules.Module1.DRIVE_MOTOR,
              Modules.Module1.STEER_ENCODER,
              Modules.Module1.STEER_OFFSET,
              -Dims.TRACKWIDTH_METERS / 2.0,
              Dims.TRACKWIDTH_METERS / 2.0,
              false);

      // module wheel positions taken from kinematics object
      final SwerveModuleConstants backRight =
          swerveModuleConstantsFactory.createModuleConstants(
              Modules.Module2.STEER_MOTOR,
              Modules.Module2.DRIVE_MOTOR,
              Modules.Module2.STEER_ENCODER,
              Modules.Module2.STEER_OFFSET,
              -Dims.TRACKWIDTH_METERS / 2.0,
              -Dims.TRACKWIDTH_METERS / 2.0,
              true);

      swerveDrivetrain =
          new SwerveDrivetrain(
              swerveDrivetrainConstants, frontLeft, frontRight, backLeft, backRight);
      swerveModules = 
          new SwerveModule[] {swerveDrivetrain.getModule(0), swerveDrivetrain.getModule(1), swerveDrivetrain.getModule(2), swerveDrivetrain.getModule(3)};
    } else {
      swerveDrivetrain = null;
    }

    // FIXME check these values to make sure they are correct
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometryToPose,
        this::getChassisSpeeds,
        null,
        Constants.Config.PATH_FOLLOWER_CONFIG,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    rotController = new PIDController(0.03, 0.001, 0.003);
    rotController.setSetpoint(0);
    rotController.setTolerance(ANGULAR_ERROR); // degrees error
    // tune pid with:
    // tab.add(rotController);

    swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getConsistentGyroscopeRotation(),
            getSwerveModulePositions(),
            // FIXME: FIXME FIXME GOOD GOD FIX ME, USE A REAL VALUE HERE
            new Pose2d(3.5, 2.2, Rotation2d.fromDegrees(0)),
            PoseEstimator.STATE_STANDARD_DEVIATIONS,
            PoseEstimator.VISION_MEASUREMENT_STANDARD_DEVIATIONS);

    zeroGyroscope();

    // tab.addNumber("target angle", () -> targetAngle);
    // tab.addNumber("current angle", () -> getGyroscopeRotation().getDegrees());
    // tab.addNumber(
    //     "angular difference",
    //     () -> -Util.relativeAngularDifference(targetAngle, getGyroscopeRotation().getDegrees()));

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      tab.addDouble("pitch", () -> swerveDrivetrain.getPigeon2().getPitch().getValueAsDouble());
      tab.addDouble("roll", () -> swerveDrivetrain.getPigeon2().getRoll().getValueAsDouble());
      tab.addDouble("x", () -> chassisSpeeds.vxMetersPerSecond);
      tab.addDouble("y", () -> chassisSpeeds.vyMetersPerSecond);
      tab.addDouble("rot", () -> chassisSpeeds.omegaRadiansPerSecond);
      tab.addDouble("relx", () -> getRobotRelativeSpeeds().vxMetersPerSecond);
      tab.addDouble("rely", () -> getRobotRelativeSpeeds().vyMetersPerSecond);
      tab.addDouble("relrot", () -> getRobotRelativeSpeeds().omegaRadiansPerSecond);
    }

    Shuffleboard.getTab("DriverView").add(field).withPosition(0, 2).withSize(8, 4);
  }

  /** Return the current pose estimation of the robot */
  public Pose2d getPose() {
    return robotPose;
  }

  /** Return the kinematics object, for constructing a trajectory */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

    /** Return current robot-relative ChassisSpeeds **/
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(swerveDrivetrain.getState().ModuleStates);
  }

  private SwerveModulePosition[] getSwerveModulePositions() {
    return Config.DISABLE_SWERVE_INIT
        ? new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
        }
        : new SwerveModulePosition[] {
          swerveModules[0].getPosition(false),
          swerveModules[1].getPosition(false),
          swerveModules[2].getPosition(false),
          swerveModules[3].getPosition(false),
        };
  }

  private Rotation2d driverGyroOffset = Rotation2d.fromDegrees(0);

  /** Sets the gyro angle to zero, resetting the forward direction */
  public void zeroGyroscope() {
    driverGyroOffset = getConsistentGyroscopeRotation();
  }

  /** Aligns gyro heading with pose estimation */
  public void smartZeroGyroscope() {
    driverGyroOffset =
        getConsistentGyroscopeRotation()
            .minus(
                swervePoseEstimator
                    .getEstimatedPosition()
                    .getRotation()
                    .plus(
                        DriverStation.getAlliance().get() == Alliance.Blue // FIXME temp fix
                            ? new Rotation2d()
                            : Rotation2d.fromDegrees(180)));
  }

  /**
   * Resets the odometry estimate to a specific pose.
   *
   * @param pose The pose to reset to.
   */
  public void resetOdometryToPose(Pose2d pose) {

    // "Zero" the driver gyro heading
    driverGyroOffset =
        getConsistentGyroscopeRotation()
            .minus(pose.getRotation())
            .plus(
                DriverStation.getAlliance().get() == Alliance.Blue // FIXME
                    ? new Rotation2d()
                    : Rotation2d.fromDegrees(180));

    swervePoseEstimator.resetPosition(
        getConsistentGyroscopeRotation(), getSwerveModulePositions(), pose);
  }

  /**
   * Gets the current angle of the robot, relative to boot position. This value will not be reset,
   * and is used for odometry.
   *
   * <p>Use this value for odometry.
   *
   * @return The current angle of the robot, relative to boot position.
   */
  public Rotation2d getConsistentGyroscopeRotation() {
    return Rotation2d.fromDegrees(Util.normalizeDegrees(-swerveDrivetrain.getPigeon2().getAngle()));
  }

  /**
   * Gets the current angle of the robot, relative to the last time zeroGyroscope() was called. This
   * is not the same as the angle of the robot on the field, which is what getPose().getRotation()
   * returns. This is the angle of the robot as the driver sees it.
   *
   * <p>Use this value for driving the robot.
   *
   * @return The current angle of the robot, relative to the last time zeroGyroscope() was called.
   */
  public Rotation2d getDriverGyroscopeRotation() {

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes
    // the angle increase.
    double angle = Util.normalizeDegrees(-swerveDrivetrain.getPigeon2().getAngle());

    // We need to subtract the offset here so that the robot drives forward based on auto
    // positioning or manual reset
    return Rotation2d.fromDegrees(angle).minus(driverGyroOffset);
  }

  public double getRotVelocity() {
    return swerveDrivetrain.getPigeon2().getRate();
  }

  /**
   * Tells the subsystem to drive, and puts the state machine in drive mode
   *
   * @param chassisSpeeds the speed of the chassis desired
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    this.mode = Modes.DRIVE;
    this.chassisSpeeds = chassisSpeeds;
  }

  public void driveAngle(Pair<Double, Double> xyInput, double targetAngle) {
    this.xyInput = xyInput;
    this.targetAngle = targetAngle;
    if (mode != Modes.DRIVE_ANGLE) rotController.reset();
    mode = Modes.DRIVE_ANGLE;
  }

  /**
   * gets the current mode of the drivebase subsystem state machine
   *
   * @return the current mode
   */
  public Modes getMode() {
    return mode;
  }

  /**
   * Angles the swerve modules in a cross shape, to make the robot hard to push. This function sets
   * the state machine to defense mode, so it only needs to be called once
   */
  public void setDefenseMode() {
    mode = Modes.DEFENSE;
  }

  /**
   * Updates the robot pose estimation for newly written module states. Should be called on every
   * periodic
   */
  private void odometryPeriodic() {
    this.robotPose =
        swervePoseEstimator.update(getConsistentGyroscopeRotation(), getSwerveModulePositions());

    VisionMeasurement measurement;
    while ((measurement = visionSubsystem.drainVisionMeasurement()) != null) {
      swervePoseEstimator.addVisionMeasurement(
          measurement.estimation().estimatedPose.toPose2d(),
          measurement.estimation().timestampSeconds,
          measurement.confidence());
    }
  }

  private void drivePeriodic() {
    swerveDrivetrain.setControl(chassisSpeedRequest.withSpeeds(chassisSpeeds));
    // SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    // for(int i = 0; i < swerveModules.length; i++) {
    //   swerveModules[i].apply(states[i], Modules.Params.driveRequestType, Modules.Params.steerRequestType);
    // }
  }

  // called in drive to angle mode
  private void driveAnglePeriodic() {
    double angularDifference =
        -Util.relativeAngularDifference(getDriverGyroscopeRotation(), targetAngle);

    double rotationValue = rotController.calculate(angularDifference);

    // we are treating this like a joystick, so -1 and 1 are its lower and upper bound
    rotationValue = MathUtil.clamp(rotationValue, -1, 1);

    // this value makes our unit-less [-1, 1] into [-max angular, max angular]
    double omegaRadiansPerSecond = rotationValue * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // initialize chassis speeds but add our desired angle
    chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xyInput.getFirst(),
            xyInput.getSecond(),
            omegaRadiansPerSecond,
            getDriverGyroscopeRotation());

    // use the existing drive periodic logic to assign to motors ect
    drivePeriodic();
  }

  @SuppressWarnings("java:S1121")
  private void defensePeriodic() {
    swerveDrivetrain.setControl(swerveBrakeRequest);
    // No need to call odometry periodic
  }

  public record RollPitch(double roll, double pitch) {
    public static RollPitch fromPigeon(Pigeon2 pigeon) {
      return new RollPitch(
          pigeon.getRoll().getValueAsDouble(), pigeon.getPitch().getValueAsDouble());
    }

    public double absRoll() {
      return Math.abs(roll);
    }

    public double absPitch() {
      return Math.abs(pitch);
    }
  }

  public RollPitch getRollPitch() {
    return RollPitch.fromPigeon(swerveDrivetrain.getPigeon2());
  }

  /**
   * Based on the current Mode of the drivebase, perform the mode-specific logic such as writing
   * outputs (may vary per mode).
   *
   * @param mode The mode to use (should use the current mode value)
   */
  public void updateModules(Modes mode) {
    if (Config.DISABLE_SWERVE_INIT) return;
    switch (mode) {
      case DRIVE -> drivePeriodic();
      case DRIVE_ANGLE -> driveAnglePeriodic();
      case DEFENSE -> defensePeriodic();
    }
  }

  /** For use in #periodic, to calculate the timestamp between motor writes */
  private double lastTimestamp = 0.0;

  @Override
  public void periodic() {
    /* Calculate time since last run and update odometry accordingly */
    final double timestamp = Timer.getFPGATimestamp();
    final double dt = timestamp - lastTimestamp;
    lastTimestamp = timestamp;

    /* get the current set-points for the drivetrain */
    Modes currentMode = getMode();
    Pose2d pose = getPose();

    field.setRobotPose(swervePoseEstimator.getEstimatedPosition());
    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      SmartDashboard.putString(
          "pose",
          String.format(
              "(%2f %2f %2f)",
              swervePoseEstimator.getEstimatedPosition().getX(),
              swervePoseEstimator.getEstimatedPosition().getY(),
              swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees()));
    }

    /* Write outputs, corresponding to our current Mode of operation */
    updateModules(currentMode);

    /* Update odometry */
    odometryPeriodic();
  }

  public static ChassisSpeeds produceChassisSpeeds(
      boolean isRobotRelativeForward,
      // boolean isRobotRelativeBackward,
      double x,
      double y,
      double rotationVelocity,
      Rotation2d currentGyroAngle) {
    if (isRobotRelativeForward) return new ChassisSpeeds(x, y, rotationVelocity);
    // if (isRobotRelativeBackward) return new ChassisSpeeds(-x, -y, rotationVelocity);
    return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotationVelocity, currentGyroAngle);
  }
}
