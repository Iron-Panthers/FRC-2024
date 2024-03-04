// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Config;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Setpoints;
import frc.robot.commands.AdvancedIntakeCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefenseModeCommand;
import frc.robot.commands.HaltDriveCommandsCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PivotAngleCommand;
import frc.robot.commands.PivotManualCommand;
import frc.robot.commands.PivotTargetLockCommand;
import frc.robot.commands.RotateAngleDriveCommand;
import frc.robot.commands.RotateVectorDriveCommand;
import frc.robot.commands.RotateVelocityDriveCommand;
import frc.robot.commands.SetRampModeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterRampUpCommand;
import frc.robot.commands.StopIntakeCommand;
import frc.robot.commands.StopShooterCommand;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.subsystems.CANWatchdogSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NetworkWatchdogSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.RGBSubsystem.RGBMessage;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.util.ControllerUtil;
import frc.util.Layer;
import frc.util.MacUtil;
import frc.util.Util;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem(visionSubsystem);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final RGBSubsystem rgbSubsystem = new RGBSubsystem();

  private final NetworkWatchdogSubsystem networkWatchdogSubsystem =
      new NetworkWatchdogSubsystem(Optional.of(rgbSubsystem));

  private final CANWatchdogSubsystem canWatchdogSubsystem = new CANWatchdogSubsystem(rgbSubsystem);

  /** controller 1 */
  private final CommandXboxController jacob = new CommandXboxController(1);
  /** controller 1 layer */
  private final Layer jacobLayer = new Layer(jacob.rightBumper());
  /** controller 0 */
  private final CommandXboxController anthony = new CommandXboxController(0);
  /** controller 0 layer */
  //   private final Layer anthonyLayer = new Layer(anthony.rightBumper());

  /** the sendable chooser to select which auto to run. */
  private final SendableChooser<Command> autoSelector;

  private GenericEntry autoDelay;
  
  private RGBMessage twoNoteMessage;

  private RGBMessage serializerNoteMessage;

  private RGBMessage acceleratorNoteMessage;

  private RGBMessage readyToShootMessage;

  private final ShuffleboardTab driverView = Shuffleboard.getTab("DriverView");

  /* drive joystick "y" is passed to x because controller is inverted */
  private final DoubleSupplier translationXSupplier =
      () -> (-modifyAxis(anthony.getLeftY()) * Drive.MAX_VELOCITY_METERS_PER_SECOND);
  private final DoubleSupplier translationYSupplier =
      () -> (-modifyAxis(anthony.getLeftX()) * Drive.MAX_VELOCITY_METERS_PER_SECOND);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // reigster commands for pathplanner
    NamedCommands.registerCommand(
        "IntakeCommand", new IntakeCommand(intakeSubsystem, shooterSubsystem, pivotSubsystem));
    NamedCommands.registerCommand("ShootCommand", new ShootCommand(shooterSubsystem));
    NamedCommands.registerCommand(
        "ShooterRampUpCommand", new ShooterRampUpCommand(shooterSubsystem));
    NamedCommands.registerCommand("SetShooterToRamping", new SetRampModeCommand(shooterSubsystem));
    NamedCommands.registerCommand("AngleAtSpeaker", new PivotAngleCommand(pivotSubsystem, 55));
    NamedCommands.registerCommand("AngleAt1", new PivotAngleCommand(pivotSubsystem, 40));
    NamedCommands.registerCommand("AngleAt2", new PivotAngleCommand(pivotSubsystem, 40));
    NamedCommands.registerCommand("AngleAtFar", new PivotAngleCommand(pivotSubsystem, 30));
    NamedCommands.registerCommand(
        "AutoAngle", new PivotTargetLockCommand(pivotSubsystem, drivebaseSubsystem));
    NamedCommands.registerCommand(
        "recenterPose1",
        DriverStation.getAlliance().get().equals(Alliance.Blue)
            ? new InstantCommand(
                () ->
                    drivebaseSubsystem.resetOdometryToPose(
                        new Pose2d(new Translation2d(1.12, 6.68), new Rotation2d(56.93))),
                drivebaseSubsystem)
            : new InstantCommand(
                () ->
                    drivebaseSubsystem.resetOdometryToPose(
                        new Pose2d(new Translation2d(15.6, 6.68), new Rotation2d(-56.93))),
                drivebaseSubsystem));
    NamedCommands.registerCommand(
        "recenterPose2",
        DriverStation.getAlliance().get().equals(Alliance.Blue)
            ? new InstantCommand(
                () ->
                    drivebaseSubsystem.resetOdometryToPose(
                        new Pose2d(new Translation2d(1.4, 5.56), new Rotation2d(0))),
                drivebaseSubsystem)
            : new InstantCommand(
                () ->
                    drivebaseSubsystem.resetOdometryToPose(
                        new Pose2d(new Translation2d(15.2, 5.56), new Rotation2d(180))),
                drivebaseSubsystem));
    NamedCommands.registerCommand(
        "recenterPose3",
        DriverStation.getAlliance().get().equals(Alliance.Blue)
            ? new InstantCommand(
                () ->
                    drivebaseSubsystem.resetOdometryToPose(
                        new Pose2d(new Translation2d(1.12, 4.36), new Rotation2d(-98.90))),
                drivebaseSubsystem)
            : new InstantCommand(
                () ->
                    drivebaseSubsystem.resetOdometryToPose(
                        new Pose2d(new Translation2d(15.6, 6.68), new Rotation2d(98.90))),
                drivebaseSubsystem));
    NamedCommands.registerCommand(
        "recenterPose4",
        DriverStation.getAlliance().get().equals(Alliance.Blue)
            ? new InstantCommand(
                () ->
                    drivebaseSubsystem.resetOdometryToPose(
                        new Pose2d(new Translation2d(1, 2.5), new Rotation2d(0))),
                drivebaseSubsystem)
            : new InstantCommand(
                () ->
                    drivebaseSubsystem.resetOdometryToPose(
                        new Pose2d(new Translation2d(15.5, 2.5), new Rotation2d(180))),
                drivebaseSubsystem));

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivebaseSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            drivebaseSubsystem,
            translationXSupplier,
            translationYSupplier,
            // anthony.rightBumper(),
            anthony.leftBumper()));

    // pivotSubsystem.setDefaultCommand(
    //     new PivotManualCommand(pivotSubsystem, () -> -jacob.getLeftY()));

    // Configure the button bindings
    configureButtonBindings();

    autoSelector = AutoBuilder.buildAutoChooser();

    SmartDashboard.putBoolean("is comp bot", MacUtil.IS_COMP_BOT);
    SmartDashboard.putBoolean("show debug data", Config.SHOW_SHUFFLEBOARD_DEBUG_DATA);
    SmartDashboard.putBoolean("don't init swerve modules", Config.DISABLE_SWERVE_INIT);

    // Create and put autonomous selector to dashboard
    setupAutonomousCommands();
  }

  /**
   * Use this method to do things as the drivers gain control of the robot. We use it to vibrate the
   * driver b controller to notice accidental swaps.
   *
   * <p>Please use this very, very sparingly. It doesn't exist by default for good reason.
   */
  public void containerTeleopInit() {
    // runs when teleop happens
    CommandScheduler.getInstance().schedule(new VibrateHIDCommand(jacob.getHID(), 5, .5));
  }

  /**
   * Use this method to do things as soon as the robot starts being used. We use it to stop doing
   * things that could be harmful or undesirable during game play--rebooting the network switch is a
   * good example. Subsystems need to be explicitly wired up to this method.
   *
   * <p>Depending on which mode the robot is enabled in, this will either be called before auto or
   * before teleop, whichever is first.
   *
   * <p>Please use this very, very sparingly. It doesn't exist by default for good reason.
   */
  public void containerMatchStarting() {
    // runs when the match starts
    networkWatchdogSubsystem.matchStarting();
    canWatchdogSubsystem.matchStarting();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // vibrate jacob controller when in layer
    jacobLayer.whenChanged(
        (enabled) -> {
          final double power = enabled ? .1 : 0;
          jacob.getHID().setRumble(RumbleType.kLeftRumble, power);
          jacob.getHID().setRumble(RumbleType.kRightRumble, power);
        });

    anthony
        .start()
        .onTrue(new InstantCommand(drivebaseSubsystem::zeroGyroscope, drivebaseSubsystem));

    // STOP INTAKE-SHOOTER
    jacob
        .x()
        .onTrue(
            new StopShooterCommand(shooterSubsystem)
                .alongWith(new StopIntakeCommand(intakeSubsystem)));
    // OUTTAKE
    jacob.rightBumper().onTrue(new OuttakeCommand(intakeSubsystem));

    // INTAKE
    anthony
        .leftBumper()
        .onTrue(new AdvancedIntakeCommand(intakeSubsystem, shooterSubsystem, pivotSubsystem));

    jacob
        .leftBumper()
        .onTrue(new AdvancedIntakeCommand(intakeSubsystem, shooterSubsystem, pivotSubsystem));

    // SHOOT
    anthony
        .rightBumper()
        .onTrue(
            new ShooterRampUpCommand(shooterSubsystem)
                .andThen(new ShootCommand(shooterSubsystem))
                .andThen(
                    new AdvancedIntakeCommand(intakeSubsystem, shooterSubsystem, pivotSubsystem)));

    // SHOOT OVERRIDE
    jacob.rightTrigger().onTrue(new ShootCommand(shooterSubsystem));

    anthony.rightStick().onTrue(new DefenseModeCommand(drivebaseSubsystem));
    anthony.leftStick().onTrue(new HaltDriveCommandsCommand(drivebaseSubsystem));
    jacob.y().onTrue(new PivotTargetLockCommand(pivotSubsystem, drivebaseSubsystem));

    anthony.povUp().onTrue(new PivotAngleCommand(pivotSubsystem, 30));
    anthony.povLeft().onTrue(new PivotAngleCommand(pivotSubsystem, 60));
    anthony.povRight().onTrue(new PivotAngleCommand(pivotSubsystem, 75));
    anthony.povDown().onTrue(new PivotAngleCommand(pivotSubsystem, 55));

    DoubleSupplier pivotManualRate = () -> modifyAxis(-jacob.getLeftY());

    new Trigger(() -> Math.abs(pivotManualRate.getAsDouble()) > 0.07)
        .onTrue(new PivotManualCommand(pivotSubsystem, pivotManualRate));

    // SOURCE
    anthony
        .y()
        .onTrue(
            new RotateAngleDriveCommand(
                    drivebaseSubsystem,
                    translationXSupplier,
                    translationYSupplier,
                    DriverStation.getAlliance().get().equals(Alliance.Blue)
                        ? Setpoints.SOURCE_DEGREES
                        : (-Setpoints.SOURCE_DEGREES))
                .alongWith(
                    new AdvancedIntakeCommand(intakeSubsystem, shooterSubsystem, pivotSubsystem)));

    // SPEAKER FROM STAGE
    anthony
        .b()
        .onTrue(
            new RotateAngleDriveCommand(
                    drivebaseSubsystem,
                    translationXSupplier,
                    translationYSupplier,
                    DriverStation.getAlliance().get().equals(Alliance.Blue)
                        ? Setpoints.SPEAKER_DEGREES
                        : (-Setpoints.SPEAKER_DEGREES))
                .alongWith(new PivotAngleCommand(pivotSubsystem, 28)));

    // AMP
    anthony
        .x()
        .onTrue(
            new RotateAngleDriveCommand(
                    drivebaseSubsystem,
                    translationXSupplier,
                    translationYSupplier,
                    DriverStation.getAlliance().get().equals(Alliance.Blue) ? 90 : (90 + 180))
                .alongWith(new PivotAngleCommand(pivotSubsystem, 80)));

    // anthony
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () ->
    //                 drivebaseSubsystem.resetOdometryToPose(
    //                     new Pose2d(new Translation2d(15.6, 6.68), new Rotation2d(-56.93))),
    //             drivebaseSubsystem));

    // SPEAKER FROM SUBWOOFER
    // anthony.a().onTrue(new PivotAngleCommand(pivotSubsystem, 56));
    new RotateAngleDriveCommand(
            drivebaseSubsystem,
            translationXSupplier,
            translationYSupplier,
            DriverStation.getAlliance().get().equals(Alliance.Blue) ? 0 : 180)
        .alongWith(new PivotAngleCommand(pivotSubsystem, 56));

    DoubleSupplier rotation =
        exponential(
            () ->
                ControllerUtil.deadband(
                    (anthony.getRightTriggerAxis() + -anthony.getLeftTriggerAxis()), .1),
            2);

    DoubleSupplier rotationVelocity =
        () -> rotation.getAsDouble() * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.8;

    new Trigger(() -> Math.abs(rotation.getAsDouble()) > 0)
        .whileTrue(
            new RotateVelocityDriveCommand(
                drivebaseSubsystem,
                translationXSupplier,
                translationYSupplier,
                rotationVelocity,
                anthony.rightBumper()));

    new Trigger(
            () ->
                Util.vectorMagnitude(anthony.getRightY(), anthony.getRightX())
                    > Drive.ROTATE_VECTOR_MAGNITUDE)
        .onTrue(
            new RotateVectorDriveCommand(
                drivebaseSubsystem,
                translationXSupplier,
                translationYSupplier,
                anthony::getRightY,
                anthony::getRightX,
                anthony.rightBumper()));

        //two note = yellow
    new Trigger(
        () -> shooterSubsystem.isBeamBreakSensorTriggered() && intakeSubsystem.isBeamBreakSensorTriggered())
        .onTrue( new InstantCommand(() -> {
        twoNoteMessage = rgbSubsystem.showMessage(Constants.Lights.Colors.YELLOW,
                    RGBSubsystem.PatternTypes.PULSE,
                    RGBSubsystem.MessagePriority.C_TWO_NOTE_WARNING);
        }, rgbSubsystem))
        .onFalse( new InstantCommand(() -> twoNoteMessage.expire()));
    
        //serializer = blue
    new Trigger(
        () -> intakeSubsystem.isBeamBreakSensorTriggered() || shooterSubsystem.isBeamBreakSensorTriggered())
        .onTrue( new InstantCommand(() -> {
        serializerNoteMessage = rgbSubsystem.showMessage(Constants.Lights.Colors.BLUE,
                    RGBSubsystem.PatternTypes.PULSE,
                    RGBSubsystem.MessagePriority.F_NOTE_IN_ROBOT);
        }, rgbSubsystem))
        .onFalse( new InstantCommand(() -> serializerNoteMessage.expire()));

        //ready to shoot = red
    new Trigger(
        () -> shooterSubsystem.isReadyToShoot() 
        && pivotSubsystem.isAtTargetDegrees())
        .onTrue( new InstantCommand(() -> {
        readyToShootMessage = rgbSubsystem.showMessage(Constants.Lights.Colors.RED,
                    RGBSubsystem.PatternTypes.PULSE,
                    RGBSubsystem.MessagePriority.D_READY_TO_SHOOT);
        }, rgbSubsystem))
        .onFalse( new InstantCommand(() -> readyToShootMessage.expire()));
  }

  /**
   * Adds all autonomous routines to the autoSelector, and places the autoSelector on Shuffleboard.
   */
  private void setupAutonomousCommands() {
    driverView.addString("NOTES", () -> "...win?\nor not.").withSize(4, 1).withPosition(7, 2);

    driverView.add("auto selector", autoSelector).withSize(4, 1).withPosition(7, 0);

    autoDelay =
        driverView
            .add("auto delay", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 15, "block increment", .1))
            .withSize(4, 1)
            .withPosition(7, 1)
            .getEntry();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    double delay = autoDelay.getDouble(0);
    return delay == 0
        ? autoSelector.getSelected()
        : new WaitCommand(delay).andThen(autoSelector.getSelected());
  }

  /**
   * applies deadband and squares axis
   *
   * @param value the axis value to be modified
   * @return the modified axis values
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = ControllerUtil.deadband(value, 0.07);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static DoubleSupplier exponential(DoubleSupplier supplier, double exponential) {
    return () -> {
      double val = supplier.getAsDouble();
      return Math.copySign(Math.pow(val, exponential), val);
    };
  }
}
