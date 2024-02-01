// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import edu.wpi.first.math.geometry.Pose2d;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX wristMotor;
  private TalonFX rollerMotorBottom;
  private TalonFX rollerMotorTop;
  private PIDController pidController;
  private double targetDegrees;
  private double wristMotorPower;
  private double currentTime;
  private double startTime;
  private double mathJunk;

  
  private final ShuffleboardTab WristTab = Shuffleboard.getTab("Wrist");

  /** Creates a new ShooterSubsystem. */
  // public ShooterSubsystem(DoubleSupplier xV, DoubleSupplier yV, DoubleSupplier x, DoubleSupplier y) {
    public ShooterSubsystem() {
    wristMotor = new TalonFX(Shooter.WRIST_MOTOR_PORT);
    // rollerMotorTop = new TalonFX(Shooter.SHOOTER_MOTOR_PORT);
    // rollerMotorBottom = new TalonFX(Shooter.SHOOTER_MOTOR_PORT);
    this.wristMotor.setPosition(0);
    wristMotor.clearStickyFaults();
    this.wristMotor.set(0);
    // rollerMotorTop.clearStickyFaults();
    // rollerMotorBottom.clearStickyFaults();
    // rollerMotorBottom.setControl(new Follower(rollerMotorTop.getDeviceID(), true));
    // rollerMotorBottom.setInverted(true);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);
    // rollerMotorTop.setNeutralMode(NeutralModeValue.Brake);
    // rollerMotorBottom.setNeutralMode(NeutralModeValue.Brake);
    pidController = new PIDController(0.1, 0, 0);
    startTime = 0;
    WristTab.addNumber("Current Motor Position", ()-> wristMotor.getPosition().getValueAsDouble());
    WristTab.addNumber("Current motor angle", this::getCurrentAngle);
    WristTab.addNumber("Motor Power", ()-> wristMotorPower);
    WristTab.addBoolean("Is at target", this::isAtTargetDegrees);
    WristTab.addNumber("Error", this::getCurrentError);
    WristTab.addNumber("target", ()-> targetDegrees);
    WristTab.addNumber("Error PID", pidController::getPositionError);
    targetDegrees = 0;
    wristMotorPower =0;
    //this.xV = xV.getAsDouble();
    //this.yV = yV.getAsDouble();
    //this.x = x.getAsDouble();
    //this.y = y.getAsDouble();
    //ShooterTab.addNumber("Target Degrees10982", () -> this.targetDegrees);
    //ShooterTab.addNumber("Motor SPeed9872", () -> wristMotorPower);
  }
  // public double speakerTargetAngle (){
  //   //Solves for the reference angle then 90 minus theta to get the angle of the robot. 
  //   //Inverse tan y/x to find theta, and the x offset is 
  //   return 90 - Math.atan(y/(x-(Shooter.NOTE_SPEED*xV)));
  // }
  // public double noteVelocity(){
  //   return Shooter.NOTE_SPEED + xV;
  // }
  private double getFeedForward() {

    // get the radians of the arm
    // getAngle() returns degrees
    double theta = Math.toRadians(getCurrentAngle());
    // get a range of 0 to 1 to multiply by feedforward.
    // when in horizontal position, value should be 1
    // when in vertical up or down position, value should be 0
    double gravityCompensation = Math.cos(theta);
    // horizontalHoldOutput is the minimum power required to hold the arm up when horizontal
    // this is a range of 0-1, in our case it was .125 throttle required to keep the arm up
    double feedForward = gravityCompensation * Shooter.HORIZONTAL_HOLD_OUTPUT;

    return 0.017;
  }
  // wrist methods
  // public void setTargetDegrees(double targetDegrees){
  //   this.targetDegrees = targetDegrees;
  //   pidController.setSetpoint(targetDegrees);
  // }
  private double getCurrentError(){
    return targetDegrees - getCurrentAngle();
  }

  private static double degreesToTicks(double angle) {
    return (angle * 360) / (Shooter.WRIST_GEAR_RATIO);
  }

  private static double rotationsToDegrees(double rotations) {
    return ((rotations / (Shooter.WRIST_GEAR_RATIO) * 360));
  }

  private double getCurrentAngle() {
    return -rotationsToDegrees(wristMotor.getPosition().getValue());
  }

  public void calculateWristTargetDegrees(Pose2d pose, double xV, double yV) {
    double g = Shooter.GRAVITY;
    double x = pose.getX();
    double y = pose.getY();
    double h = Shooter.SPEAKER_HEIGHT;
    // difference between distance to speaker now and after 1 second to find v to speaker
    double velocityToSpeaker = (Math.pow((x-Shooter.SPEAKER_X),2)+Math.pow((y-Shooter.SPEAKER_Y),2)-Math.pow((xV-Shooter.SPEAKER_X),2)+Math.pow((yV-Shooter.SPEAKER_Y),2));
    double v = Shooter.NOTE_SPEED+velocityToSpeaker;
    //iterates 3 times to find the angle. Finds time to get to the height, subtracts gravity, 
    //finds how off the height is, adds more height on next iteration to the old height, iterates 
    //3 times, is very close to the limit. 
    double y1 = Math.pow(((Math.sqrt(x*x)+(h*h))/v),2)*g*0.5;
    double y2 = Math.pow(((Math.sqrt(x*x)+((y1+h)*(y1+h)))/v),2)*g*0.5;
    double y3 = Math.pow(((Math.sqrt(x*x)+((y2+h)*(y2+h)))/v),2)*g*0.5;
    targetDegrees =180/3.14159*(Math.atan((y3+h)/x));
    pidController.setSetpoint(targetDegrees);
  }
  // other methods
  // public double getCurrentTime() {
  //   return Timer.getFPGATimestamp();
  // }

  public boolean isAtTargetDegrees() {
    return Math.abs(getCurrentAngle() - targetDegrees) < 1;
  }
  private boolean isSensorTriggered(){
    //if is triggered return true
    return false;
  }
  
  public boolean isDone(Pose2d pose) {
     return isSensorTriggered() || pose.getX()>8.4;
  }
  
  @Override
  public void periodic() {
      wristMotorPower = pidController.calculate(getCurrentAngle());
      wristMotor.set(-MathUtil.clamp(wristMotorPower+getFeedForward(), -0.5, 0.5));//you allways need to incorperate feed foreward
    // else{
    //   if (startTime == 0){
    //     startTime = getCurrentTime();
    //   }
    //   rollerMotorTop.set(Shooter.ROLLER_MOTOR_POWER);
    // }

    // This method will be called once per scheduler run
  }
}
