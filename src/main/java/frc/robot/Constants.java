// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.util.MacUtil.IS_COMP_BOT;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.Drive.Dims;
import frc.robot.subsystems.IntakeSubsystem.IntakePowers;
import frc.robot.subsystems.NetworkWatchdogSubsystem.IPv4;
import frc.robot.subsystems.RGBSubsystem.RGBColor;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem.TagCountDeviation;
import frc.robot.subsystems.VisionSubsystem.UnitDeviationParams;
import frc.util.CAN;
import java.nio.file.Path;
import java.util.List;
import java.util.Set;

@SuppressWarnings("java:S1118")
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class Config {
    // maybe tune PID values?
    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG =
        new HolonomicPathFollowerConfig(
            new PIDConstants(20, 0, 0),
            new PIDConstants(10, 0, 0),
            Drive.MAX_VELOCITY_METERS_PER_SECOND,
            Math.sqrt(Math.pow(Dims.TRACKWIDTH_METERS, 2) * 2),
            new ReplanningConfig());

    /** turn this off before comp. */
    public static final boolean SHOW_SHUFFLEBOARD_DEBUG_DATA = true;

    /** turn this off! only use on practice eboard testing. */
    public static final boolean DISABLE_SWERVE_INIT = false;

    /** keep this on for pigeon, disable if absolutely necessary */
    public static final boolean FLIP_GYROSCOPE = true;

    /** def turn this off unless you are using it, generates in excess of 100k rows for a match. */
    public static final boolean WRITE_APRILTAG_DATA = false;

    public static final Path APRILTAG_DATA_PATH =
        Filesystem.getDeployDirectory().toPath().resolve("poseEstimationsAtDistances.csv");
    public static final double REAL_X = 0.0;
    public static final double REAL_Y = 0.0;
  }

  public static final class Drive {
    public static final int PIGEON_PORT = 0; // placeholder
    public static final String SWERVE_CANBUS = "rio"; // placeholder

    // max voltage delivered to drivebase
    // supposedly useful to limit speed for testing
    public static final double MAX_VOLTAGE = 12.0;
    // maximum velocity
    // FIXME measure this value experimentally
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0 // falcon 500 free speed rpm
            / 60.0
            * 0.10033
            * (1 / 6.12) // mk4i l3 16t falcon drive reduction (sourced from adrian)
            * Math.PI;
    // theoretical value
    // FIXME measure and validate experimentally
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0)
            * .5;

    /** the maximum amount of angular error pid loops will tolerate for rotation */
    public static final double ANGULAR_ERROR = 1.0;
    /** the minimum magnitude of the right stick for it to be used as a new rotation angle */
    public static final double ROTATE_VECTOR_MAGNITUDE = .7;

    public static final class Dims {
      // FIXME validate with hardware
      public static final double TRACKWIDTH_METERS =
          .5207; // 20.5 inches (source: cad) converted to meters
      public static final double WHEELBASE_METERS = TRACKWIDTH_METERS; // robot is square

      public static final double BUMPER_WIDTH_METERS_X = .9779;
      public static final double BUMPER_WIDTH_METERS_Y = .8382;
    }

    /*
     module layout:
        |──────
     |->│#   ##steer motor
     │  │  ##cancoder
     │  │##drive motor
     module number

     steer is always left
     from corner perspective

     robot visualization:
    |──────────────────────|
    │2   10          04   1│
    │  25              24  │
    │11     S      D     03│
    │     D          S     │
    │                      │
    │                      │
    │     S          D     │
    │       D      S       │
    │12    |────────|    02│
    │  26  │        │  27  │
    │3   13│  batt  │01   4│
    |──────┴───┬┬───┴──────|
               ││
               ││
               ▼▼
         software front
     */

    public static final class Modules {
      public static final class Params {
        public static final double WHEEL_RADIUS = 2; // also in INCHES
        public static final double COUPLING_GEAR_RATIO = 3.125;
        public static final double DRIVE_GEAR_RATIO = 5.357142857142857;
        public static final double STEER_GEAR_RATIO = 21.428571428571427;
        public static final Slot0Configs DRIVE_MOTOR_GAINS =
            new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0.32).withKV(0.11).withKA(0);
        public static final Slot0Configs STEER_MOTOR_GAINS =
            new Slot0Configs().withKP(11).withKI(0).withKD(0).withKS(0.4).withKV(0.6).withKA(0);
        public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
            ClosedLoopOutputType.Voltage;
        public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
            ClosedLoopOutputType.Voltage;
        public static final SteerFeedbackType FEEDBACK_SOURCE = SteerFeedbackType.FusedCANcoder;
        public static final double SPEED_TWELVE_VOLTS = MAX_VELOCITY_METERS_PER_SECOND;
        public static final double SLIP_CURRENT = 0; // optional, unused rn
        public static final boolean STEER_MOTOR_INVERTED = true;

        public static final DriveRequestType driveRequestType = DriveRequestType.OpenLoopVoltage;
        public static final SteerRequestType steerRequestType = SteerRequestType.MotionMagic;
      }

      public static final class Module1 { // back left
        public static final int DRIVE_MOTOR = CAN.at(4, "module 1 drive motor");
        public static final int STEER_MOTOR = CAN.at(3, "module 1 steer motor");
        public static final int STEER_ENCODER = CAN.at(24, "module 1 steer encoder");

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? 0.41748046875 // comp bot offset
                : 0.0595703125; // practice bot offset
      }

      public static final class Module2 { // back right
        public static final int DRIVE_MOTOR = CAN.at(11, "module 2 drive motor");
        public static final int STEER_MOTOR = CAN.at(10, "module 2 steer motor");
        public static final int STEER_ENCODER = CAN.at(25, "module 2 steer encoder");

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -0.3920898437 // comp bot offset
                : 0.262451171875; // practice bot offset
      }

      public static final class Module3 { // front right
        public static final int DRIVE_MOTOR = CAN.at(13, "module 3 drive motor");
        public static final int STEER_MOTOR = CAN.at(12, "module 3 steer motor");
        public static final int STEER_ENCODER = CAN.at(26, "module 3 steer encoder");

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? 0.22216796875 // comp bot offset
                : -0.20825195312; // practice bot offset
      }

      public static final class Module4 { // front left
        public static final int DRIVE_MOTOR = CAN.at(2, "module 4 drive motor");
        public static final int STEER_MOTOR = CAN.at(1, "module 4 steer motor");
        public static final int STEER_ENCODER = CAN.at(27, "module 4 steer encoder");

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? 0.31127929687 // comp bot offset
                : -0.3564453125 + 180; // practice bot offset
      }
    }

    public static final class Setpoints {
      public static final Translation2d SPEAKER = new Translation2d(0, 5.5);

      public static final int SOURCE_DEGREES = 39;
      public static final int SPEAKER_DEGREES = 11;
      public static final int EPSILON = 3;
    }
  }

  public static final class Intake {
    public static final class Ports {
      public static final int INTAKE_MOTOR_PORT = 15;
      public static final int SERIALIZER_MOTOR_PORT = 16;
      public static final int INTAKE_SENSOR_PORT = 9;
    }

    public static final class Modes {
      public static final IntakePowers INTAKE = new IntakePowers(.95, .5);
      public static final IntakePowers HOLD = new IntakePowers(0, 0d);
      public static final IntakePowers REVERSE = new IntakePowers(-.5, -.5);
    }

    public static final boolean IS_BEAMBREAK = true;
  }

  public static final class Shooter {
    public static final class Ports {
      public static final int TOP_SHOOTER_MOTOR_PORT = 20;
      public static final int BOTTOM_SHOOTER_MOTOR_PORT = 19;
      public static final int ACCELERATOR_MOTOR_PORT = 17;
      public static final int BEAM_BREAK_SENSOR_PORT = 8;
    }

    public static final class Modes {
      public static final ShooterSubsystem.ShooterPowers INTAKE =
          new ShooterSubsystem.ShooterPowers(76, 1, .15);
      public static final ShooterSubsystem.ShooterPowers IDLE =
          new ShooterSubsystem.ShooterPowers(0, 0, 0);
      public static final ShooterSubsystem.ShooterPowers RAMP_SPEAKER =
          new ShooterSubsystem.ShooterPowers(76, 1, 0);
      public static final ShooterSubsystem.ShooterPowers RAMP_AMP_BACK =
          new ShooterSubsystem.ShooterPowers(25, 0.4, 0);
      public static final ShooterSubsystem.ShooterPowers RAMP_AMP_FRONT =
          new ShooterSubsystem.ShooterPowers(10, 2.5, 0);
      public static final ShooterSubsystem.ShooterPowers SHOOT_SPEAKER =
          new ShooterSubsystem.ShooterPowers(76, 1, .5);
      public static final ShooterSubsystem.ShooterPowers TARGET_LOCK =
          new ShooterSubsystem.ShooterPowers(0, 1, 0);
      public static final ShooterSubsystem.ShooterPowers SHOOT_AMP_BACK =
          new ShooterSubsystem.ShooterPowers(25, 0.4, .5);
      public static final ShooterSubsystem.ShooterPowers SHOOT_AMP_FORWARD =
          new ShooterSubsystem.ShooterPowers(10, 2.5, .5);
      public static final ShooterSubsystem.ShooterPowers ACCEL_SECURE =
          new ShooterSubsystem.ShooterPowers(76, 1, 0.5);
      public static final ShooterSubsystem.ShooterPowers MAINTAIN_VELOCITY =
          new ShooterSubsystem.ShooterPowers(40, 1, 0);
    }

    public static final Slot0Configs ROLLER_PID_CONFIG =
        new Slot0Configs().withKP(0.2).withKS(0.23).withKV(0.24);

    public static final double SHOOTER_VELOCITY_THRESHOLD = 30;
  }

  public static final class Pivot {
    public static final class Ports {
      public static final int PIVOT_MOTOR_PORT = 18;
      public static final int CANCODER_PORT = 28;
      public static final int INDUCTIVE_PROXIMITY_SENSOR_PORT = 40;
    }

    public static final class MotorConfigs {
      public static final MagnetSensorConfigs CANCODER_MAGNET_SENSOR =
          new MagnetSensorConfigs()
              .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
              .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
              .withMagnetOffset(PIVOT_CANCODER_OFFSET);
      public static final CANcoderConfiguration CANCODER_CONFIG =
          new CANcoderConfiguration().withMagnetSensor(CANCODER_MAGNET_SENSOR);

      public static final FeedbackConfigs PIVOT_FEEDBACK =
          new FeedbackConfigs()
              .withFeedbackRemoteSensorID(Ports.CANCODER_PORT)
              .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
              .withSensorToMechanismRatio(1.0)
              .withRotorToSensorRatio(PIVOT_GEAR_RATIO);
      public static final SoftwareLimitSwitchConfigs PIVOT_SOFTWARE_LIMIT =
          new SoftwareLimitSwitchConfigs()
              .withForwardSoftLimitThreshold(0.25)
              .withReverseSoftLimitThreshold(-0.015)
              .withForwardSoftLimitEnable(false)
              .withReverseSoftLimitEnable(false);
      public static final VoltageConfigs PIVOT_VOLTAGE =
          new VoltageConfigs().withPeakForwardVoltage(5).withPeakReverseVoltage(-5);
      public static final TalonFXConfiguration PIVOT_CONFIG =
          new TalonFXConfiguration()
              .withFeedback(PIVOT_FEEDBACK)
              .withSoftwareLimitSwitch(PIVOT_SOFTWARE_LIMIT)
              .withVoltage(PIVOT_VOLTAGE);
    }

    public static final class Setpoints {
      public static final int MINIMUM_ANGLE = 15;
      public static final int MAXIMUM_ANGLE = 125;

      public static final int MINIMUM_SAFE_THRESHOLD = 15;
      public static final int MAXIMUM_SAFE_THRESHOLD = 80;

      public static final int SPEAKER_ANGLE = 30;
    }

    public static final int EPSILON = 2;

    public static final double PIVOT_CANCODER_OFFSET = -0.625977 + (0.070139 - 0.031250);
    public static final double PIVOT_GEAR_RATIO =
        (60 / 8) * (60 / 16) * (72 / 15); // FIXME placeholder values
    public static final double CENTER_OF_ROBOT_TO_BUMPER = 0.41275;

    public static final Pose2d RED_SPEAKER_POSE = new Pose2d(16.45, 5.5, null);
    public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(0, 5.5, null);

    public static final double GRAVITY = 9.80665; // meters per second

    public static final double GRAVITY_VOLTAGE = 0.4;
    public static final double PIVOT_MAX_VOLTAGE = 3.5;
  }

  public static final class Vision {
    public static record VisionSource(String name, Transform3d robotToCamera) {}

    public static final List<VisionSource> VISION_SOURCES =
        List.of(
            new VisionSource(
                "frontCam",
                new Transform3d(
                    new Translation3d(
                        -0.305, // front/back
                        -0.2286, // left/right
                        -0.2159 // up/down
                        ),
                    new Rotation3d(
                        0,
                        Math.toRadians(30), // angle up/down
                        Math.toRadians(180)))),
            new VisionSource(
                "backCam",
                new Transform3d(
                    new Translation3d(
                        -0.2796, // front/back
                        -0.2286, // left/right
                        -0.2159 // up/down
                        ),
                    new Rotation3d(
                        0,
                        Math.toRadians(30), // angle up/down
                        Math.toRadians(180)))),
            new VisionSource(
                "backUpCam",
                new Transform3d(
                    new Translation3d(
                        -0.2796 + 0.0254, // front/back
                        0.2286, // left/right
                        -0.2159 // up/down
                        ),
                    new Rotation3d(
                        0,
                        Math.toRadians(30), // angle up/down
                        Math.toRadians(180)))));

    public static final int THREAD_SLEEP_DURATION_MS = 5;
  }

  public static final class PoseEstimator {
    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N1(),
            0.1, // x
            0.1, // y
            0.1 // theta
            );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians.
     *
     * <p>These are not actually used anymore, but the constructor for the pose estimator wants
     * them. This value is calculated dynamically using the below list.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        MatBuilder.fill(
            Nat.N3(),
            Nat.N1(),
            1, // x
            1, // y
            1 * Math.PI // theta
            );

    public static final double POSE_AMBIGUITY_CUTOFF = .05;

    public static final List<TagCountDeviation> TAG_COUNT_DEVIATION_PARAMS =
        List.of(
            // 1 tag
            new TagCountDeviation(
                new UnitDeviationParams(.25, .4, .9),
                new UnitDeviationParams(.35, .5, 1.2),
                new UnitDeviationParams(.5, .7, 1.5)),

            // 2 tags
            new TagCountDeviation(
                new UnitDeviationParams(.35, .1, .4), new UnitDeviationParams(.5, .7, 1.5)),

            // 3+ tags
            new TagCountDeviation(
                new UnitDeviationParams(.25, .07, .25), new UnitDeviationParams(.15, 1, 1.5)));

    /** about one inch */
    public static final double DRIVE_TO_POSE_XY_ERROR_MARGIN_METERS = .025;

    public static final double DRIVE_TO_POSE_THETA_ERROR_MARGIN_DEGREES = 2;

    public static final List<Set<Integer>> POSSIBLE_FRAME_FID_COMBOS =
        List.of(Set.of(1, 2, 3, 4), Set.of(5, 6, 7, 8));

    public static final int MAX_FRAME_FIDS = 4;
  }

  public static final class NetworkWatchdog {
    /** The IP addresses to ping for testing bridging, on the second vlan. */
    public static final List<IPv4> TEST_IP_ADDRESSES =
        List.of(IPv4.internal(17), IPv4.internal(18), IPv4.internal(19));

    /**
     * The number of ms (sleep delta using oshi system uptime) to wait before beginning to ping the
     * test IP.
     */
    public static final int BOOT_SCAN_DELAY_MS = 50_000;

    /** The number of seconds for ping to wait before giving up on reaching a device. */
    public static final int PING_TIMEOUT_SECONDS = 2;

    /** The number of ms to wait before retrying successful health checks. */
    public static final int HEALTHY_CHECK_INTERVAL_MS = 5_000;

    /**
     * The number of ms to leave the switching pdh port off before turning it back on as part of
     * rebooting the network switch.
     */
    public static final int REBOOT_DURATION_MS = 1_000;

    /**
     * The number of ms to wait before rerunning health checks after a failed check which triggered
     * switch reboot.
     */
    public static final int SWITCH_POWERCYCLE_SCAN_DELAY_MS = 25_000;
  }

  public static final class CANWatchdog {
    public static final int SCAN_DELAY_MS = 100;
  }

  public static final class Lights {
    public static final int CANDLE_ID = 34;
    public static final int NUM_LEDS =
        89
            // 8 inside the candle, 8 more for balance
            + 8 * 2;

    public static final class Colors {
      public static final RGBColor YELLOW = new RGBColor(255, 107, 0);
      public static final RGBColor PURPLE = new RGBColor(127, 0, 127);
      public static final RGBColor RED = new RGBColor(255, 0, 0);
      public static final RGBColor ORANGE = new RGBColor(255, 35, 0);
      public static final RGBColor BLUE = new RGBColor(0, 0, 255);
      public static final RGBColor PINK = new RGBColor(250, 35, 100);
      public static final RGBColor MINT = new RGBColor(55, 255, 50);
      public static final RGBColor TEAL = new RGBColor(0, 255, 255);
      public static final RGBColor WHITE = new RGBColor(255, 255, 255);
    }
  }

  public static final class AutoAlign {

    public static final double FIELD_WIDTH = 16.54;

    // Blue team:
    // pose angles
    public static final int BOTTOM_MID_ANGLE = 148;
    public static final int STAGE_ANGLE = 0;
    public static final int TOP_MID_ANGLE = 0;
    public static final int AMP_ANGLE = 0;

    // These poses have not been verified
    public static final Pose2d BLUE_AMP =
        new Pose2d(2.75, 7.31, Rotation2d.fromDegrees(-AMP_ANGLE + 180));
    public static final Pose2d BLUE_TOP_MID =
        new Pose2d(5.8, 7.0, Rotation2d.fromDegrees(-TOP_MID_ANGLE + 180));

    // These poses have been verified
    public static final Pose2d BLUE_STAGE =
        new Pose2d(4.17, 4.74, Rotation2d.fromDegrees(-STAGE_ANGLE + 180));
    public static final Pose2d BLUE_BOTTOM_MID =
        new Pose2d(5.84, 1.18, Rotation2d.fromDegrees(-BOTTOM_MID_ANGLE + 180));

    // Red team:
    // These poses have not been verified
    public static final Pose2d RED_AMP =
        new Pose2d(FIELD_WIDTH - 2.75, 7.31, Rotation2d.fromDegrees(-AMP_ANGLE));
    public static final Pose2d RED_TOP_MID =
        new Pose2d(FIELD_WIDTH - 5.8, 7.0, Rotation2d.fromDegrees(-TOP_MID_ANGLE));

    // These poses have been verified
    public static final Pose2d RED_STAGE =
        new Pose2d(FIELD_WIDTH - 4.17, 4.74, Rotation2d.fromDegrees(-STAGE_ANGLE));
    public static final Pose2d RED_BOTTOM_MID =
        new Pose2d(FIELD_WIDTH - 5.84, 1.18, Rotation2d.fromDegrees(-BOTTOM_MID_ANGLE));

    public static final double BOTTOM_MID_TARGET_ANGLE = 23.5;
    public static final double TOP_MID_TARGET_ANGLE = 0;
    public static final double STAGE_TARGET_ANGLE = 0;
    public static final double AMP_TARGET_ANGLE = 0;
    public static final PathConstraints CONSTRAINTS =
        new PathConstraints(3, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
  }
}
