// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.util.MacUtil.IS_COMP_BOT;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.Drive.Dims;
import frc.robot.subsystems.NetworkWatchdogSubsystem.IPv4;
import frc.robot.subsystems.RGBSubsystem.RGBColor;
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
    /** turn this off before comp. */
    public static final boolean RUN_PATHPLANNER_SERVER =
        // never run pathplanner server in simulation, it will fail unit tests (???)
        Config.SHOW_SHUFFLEBOARD_DEBUG_DATA
            && HALUtil.getHALRuntimeType() != HALUtil.RUNTIME_SIMULATION;

    // FIXME: These values should be replaced with actual values
    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG =
        new HolonomicPathFollowerConfig(
            new PIDConstants(5, 0, 0),
            new PIDConstants(5, 0, 0),
            Drive.MAX_VELOCITY_METERS_PER_SECOND,
            Math.sqrt(Math.pow(Dims.BUMPER_WIDTH_METERS, 2) * 2),
            new ReplanningConfig());

    /** turn this off before comp. */
    public static final boolean SHOW_SHUFFLEBOARD_DEBUG_DATA = true;

    /** turn this off! only use on practice eboard testing. */
    public static final boolean DISABLE_SWERVE_INIT = false;

    /** def turn this off unless you are using it, generates in excess of 100k rows for a match. */
    public static final boolean WRITE_APRILTAG_DATA = false;

    public static final Path APRILTAG_DATA_PATH =
        Filesystem.getDeployDirectory().toPath().resolve("poseEstimationsAtDistances.csv");
    public static final double REAL_X = 0.0;
    public static final double REAL_Y = 0.0;
  }

  public static final class Drive {
    public static final int PIGEON_PORT = 0; // FIXME placeholder
    public static final String SWERVE_CANBUS = "rio"; // placeholder

    // max voltage delivered to drivebase
    // supposedly useful to limit speed for testing
    public static final double MAX_VOLTAGE = 12.0;
    // maximum velocity
    // FIXME measure this value experimentally
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0 // falcon 500 free speed rpm
            / 60.0
            //      * SdsModuleConfigurations.MK4_L2.getDriveReduction()
            //      * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
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

      public static final double BUMPER_WIDTH_METERS = .851;
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
        // FIXME ALL PLACEHOLDERS
        /* Currently use L2 gearing for alphabot, will use L3 for comp bot? Not decided? Check w/ engie */
        public static final double WHEEL_RADIUS = 2; // FIXME
        public static final double COUPLING_GEAR_RATIO = 3.5714285714285716; // optional
        public static final double DRIVE_GEAR_RATIO = 6.746031746031747; // unsure?
        public static final double STEER_GEAR_RATIO = 12.8; // FIXME
        public static final Slot0Configs DRIVE_MOTOR_GAINS =
            new Slot0Configs()
                .withKP(0.2)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0)
                .withKA(0); // placeholder
        public static final Slot0Configs STEER_MOTOR_GAINS =
            new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
        public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
            ClosedLoopOutputType.TorqueCurrentFOC;
        public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
            ClosedLoopOutputType.TorqueCurrentFOC;
        public static final SteerFeedbackType FEEDBACK_SOURCE =
            SteerFeedbackType.FusedCANcoder; // dunno if this is the best option
        public static final double SPEED_TWELVE_VOLTS = 0;
        public static final double SLIP_CURRENT = 0; // optional
      }

      public static final class Module1 { // historically front right
        public static final int DRIVE_MOTOR = CAN.at(4, "module 1 drive motor");
        public static final int STEER_MOTOR = CAN.at(3, "module 1 steer motor");
        public static final int STEER_ENCODER = CAN.at(24, "module 1 steer encoder");

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(0) // comp bot offset
                : -Math.toRadians(0); // practice bot offset
      }

      public static final class Module2 { // historically front left
        public static final int DRIVE_MOTOR = CAN.at(11, "module 2 drive motor");
        public static final int STEER_MOTOR = CAN.at(10, "module 2 steer motor");
        public static final int STEER_ENCODER = CAN.at(25, "module 2 steer encoder");

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(0) // comp bot offset
                : -Math.toRadians(0); // practice bot offset
      }

      public static final class Module3 { // historically back left
        public static final int DRIVE_MOTOR = CAN.at(13, "module 3 drive motor");
        public static final int STEER_MOTOR = CAN.at(12, "module 3 steer motor");
        public static final int STEER_ENCODER = CAN.at(26, "module 3 steer encoder");

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(0) // comp bot offset
                : -Math.toRadians(0); // practice bot offset
      }

      public static final class Module4 { // historically back right
        public static final int DRIVE_MOTOR = CAN.at(2, "module 4 drive motor");
        public static final int STEER_MOTOR = CAN.at(1, "module 4 steer motor");
        public static final int STEER_ENCODER = CAN.at(27, "module 4 steer encoder");

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(0) // comp bot offset
                : -Math.toRadians(0); // practice bot offset
      }
    }
  }

  public static final class Vision {
    public static record VisionSource(String name, Transform3d robotToCamera) {}

    public static final List<VisionSource> VISION_SOURCES =
        List.of(
            new VisionSource(
                "frontCam",
                new Transform3d(
                    new Translation3d(
                        0.23749, // front/back
                        0.2403348, // left/right
                        0.7973822 // up/down
                        ),
                    new Rotation3d(
                        0,
                        Math.toRadians(-11.5), // angle up/down
                        0))),
            new VisionSource(
                "backCam",
                new Transform3d(
                    new Translation3d(
                        0, // front/back
                        -0.212725, // left/right
                        0.6470142 // up/down
                        ),
                    new Rotation3d(0, Math.toRadians(17), Math.PI))));

    public static final int THREAD_SLEEP_DURATION_MS = 5;
  }

  public static final class PoseEstimator {
    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS =
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
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
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                // if these numbers are less than one, multiplying will do bad things
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

  public static final class Intake {
    public static final class Ports{
        public static final int RIGHT_INTAKE_MOTOR_PORT = 1;
        public static final int LEFT_INTAKE_MOTOR_PORT = 2;
        public static final int SERIALIZER_MOTOR_PORT = 20;
        public static final int INTAKE_SENSOR_PORT = 3;
    }

    public static final double INTAKE_MOTOR_SPEED = 0.25;
    public static final double OUTTAKE_MOTOR_SPEED = 0.25;
    public static final double HOLD_MOTOR_SPEED = 0;

    public static final boolean IS_SERIALIZER_INVERTED = true;
  }
}
