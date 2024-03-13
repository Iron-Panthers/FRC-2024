// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Drive.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Config;
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
  private ApplyChassisSpeeds chassisSpeedRequest =
      new ApplyChassisSpeeds()
          .withDriveRequestType(Modules.Params.driveRequestType)
          .withSteerRequestType(Modules.Params.steerRequestType);
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

  /* adds swerve module values to shuffleboard
   * @param title title of the module layout
   * @param pos position of module (and layout), 0-3 corresponds to fL, fR, bL, bR
   * @param swerveDrive
   * @param shuffleboardTab
   */
  private void addSwerveShuffleboard(
      String title, int pos, SwerveModule[] swerveModules, ShuffleboardTab shuffleboardTab) {
    shuffleboardTab
        .getLayout(title, BuiltInLayouts.kList)
        .withSize(2, 1)
        .withPosition(pos * 2, 0)
        .addDouble(
            "absolute encoder rotations",
            () -> swerveModules[pos].getCANcoder().getAbsolutePosition().getValue());
  }

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
              .withSpeedAt12VoltsMps(Modules.Params.SPEED_TWELVE_VOLTS)
              .withSteerMotorInverted(Modules.Params.STEER_MOTOR_INVERTED);

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
          new SwerveModule[] {
            swerveDrivetrain.getModule(0),
            swerveDrivetrain.getModule(1),
            swerveDrivetrain.getModule(2),
            swerveDrivetrain.getModule(3)
          };
    } else {
      swerveDrivetrain = null;
    }

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometryToPose,
        this::getRobotRelativeSpeeds,
        this::drive,
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

    rotController = new PIDController(0.0179, 0, 0);
    rotController.setSetpoint(0);
    rotController.setTolerance(ANGULAR_ERROR); // degrees error

    zeroGyroscope();

    swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getConsistentGyroscopeRotation(),
            getSwerveModulePositions(),
            // FIXME: USE A REAL VALUE HERE
            new Pose2d(3.5, 2.2, Rotation2d.fromDegrees(0)),
            PoseEstimator.STATE_STANDARD_DEVIATIONS,
            PoseEstimator.VISION_MEASUREMENT_STANDARD_DEVIATIONS);

    if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
      tab.addDouble("x", () -> chassisSpeeds.vxMetersPerSecond);
      tab.addDouble("y", () -> chassisSpeeds.vyMetersPerSecond);
      tab.addDouble("rot", () -> chassisSpeeds.omegaRadiansPerSecond);
      tab.addDouble("relx", () -> getRobotRelativeSpeeds().vxMetersPerSecond);
      tab.addDouble("rely", () -> getRobotRelativeSpeeds().vyMetersPerSecond);
      tab.addDouble("relrot", () -> getRobotRelativeSpeeds().omegaRadiansPerSecond);
      tab.addDouble("targetAngle", () -> targetAngle);
      tab.addDouble("currentGyroAngle", () -> getDriverGyroscopeRotation().getDegrees());
      tab.addDouble("consistent gyro", () -> getConsistentGyroscopeRotation().getDegrees());
      tab.addDouble(
          "angular difference",
          () ->
              -Util.relativeAngularDifference(getDriverGyroscopeRotation().times(-1), targetAngle));

      addSwerveShuffleboard("module 4", 0, swerveModules, tab);
      addSwerveShuffleboard("module 3", 1, swerveModules, tab);
      addSwerveShuffleboard("module 1", 2, swerveModules, tab);
      addSwerveShuffleboard("module 2", 3, swerveModules, tab);
    }

    Shuffleboard.getTab("DriverView").add(field).withPosition(0, 0).withSize(8, 5);
  }

  public boolean isAtTargetAngle() {
    return Util.epsilonEquals(getPose().getRotation().getDegrees(), targetAngle, Setpoints.EPSILON);
  }
  /** Return the current pose estimation of the robot */
  public Pose2d getPose() {
    return robotPose;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  /** Return current robot-relative ChassisSpeeds * */
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
                        DriverStation.getAlliance().get() == Alliance.Blue
                            ? new Rotation2d()
                            : Rotation2d.fromDegrees(180)));
  }

  /**
   * Resets the odometry estimate to a specific pose.
   *
   * @param pose2d The pose to reset to.
   */
  public void resetOdometryToPose(Pose2d pose2d) {
    // "Zero" the driver gyro heading
    driverGyroOffset =
        getConsistentGyroscopeRotation()
            .minus(pose2d.getRotation())
            .plus(
                DriverStation.getAlliance().get() == Alliance.Blue
                    ? new Rotation2d()
                    : Rotation2d.fromDegrees(180));

    swervePoseEstimator.resetPosition(
        getConsistentGyroscopeRotation(), getSwerveModulePositions(), pose2d);
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
    if (Config.FLIP_GYROSCOPE) {
      return Rotation2d.fromDegrees(
          Util.normalizeDegrees(-swerveDrivetrain.getPigeon2().getAngle()));
    } else {
      return Rotation2d.fromDegrees(
          Util.normalizeDegrees(swerveDrivetrain.getPigeon2().getAngle()));
    }
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
    Rotation2d angle = getConsistentGyroscopeRotation();

    // We need to subtract the offset here so that the robot drives forward based on auto
    // positioning or manual reset
    return angle.minus(driverGyroOffset);
  }

  public double getRotVelocity() {
    return swerveDrivetrain.getPigeon2().getRate();
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    this.chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getDriverGyroscopeRotation());
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

    visionSubsystem.setRobotPose(this.getPose());

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
  }

  // called in drive to angle mode
  private void driveAnglePeriodic() {
    double angularDifference =
        -Util.relativeAngularDifference(getDriverGyroscopeRotation().times(-1), targetAngle);

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

  @Override
  public void periodic() {
    /* get the current set-points for the drivetrain */
    Modes currentMode = getMode();

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
      double x,
      double y,
      double rotationVelocity,
      Rotation2d currentGyroAngle) {
    if (isRobotRelativeForward) return new ChassisSpeeds(x, y, rotationVelocity);
    return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotationVelocity, currentGyroAngle);
  }
}
