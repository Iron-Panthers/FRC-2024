// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.RGBSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.RGBSubsystem.RGBMessage;

public class RGBCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private RGBSubsystem rgbSubsystem;
  private PivotSubsystem pivotSubsystem;
  private Optional <RGBMessage> noteInRobotMsg;
  private Optional <RGBMessage> readyToShootMsg;
  private Optional <RGBMessage> twoNoteMsg;

  

  /** Creates a new RGBCommand. */
  
  public RGBCommand(ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      RGBSubsystem rgbSubsystem,
      PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(rgbSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.rgbSubsystem = rgbSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    twoNoteMsg = Optional.empty();
    readyToShootMsg = Optional.empty();
    noteInRobotMsg = Optional.empty();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
            //two note = yellow

      if (shooterSubsystem.isBeamBreakSensorTriggered() 
      && intakeSubsystem.isBeamBreakSensorTriggered() 
      && twoNoteMsg.isEmpty()){
        twoNoteMsg =
            Optional.of(rgbSubsystem.showMessage(Constants.Lights.Colors.YELLOW,
        RGBSubsystem.PatternTypes.PULSE,
        RGBSubsystem.MessagePriority.C_TWO_NOTE_WARNING));      }
      else if (!(shooterSubsystem.isBeamBreakSensorTriggered() 
      && intakeSubsystem.isBeamBreakSensorTriggered())){
        twoNoteMsg.ifPresent(RGBMessage::expire);
        twoNoteMsg = Optional.empty();
      }
      
    
        //serializer = blue
      if ((intakeSubsystem.isBeamBreakSensorTriggered()
      ||shooterSubsystem.isBeamBreakSensorTriggered())
      && noteInRobotMsg.isEmpty()){ /*|| shooterSubsystem.isBeamBreakSensorTriggered()*/
        noteInRobotMsg =
            Optional.of(rgbSubsystem.showMessage(Constants.Lights.Colors.BLUE,
        RGBSubsystem.PatternTypes.PULSE,
        RGBSubsystem.MessagePriority.F_NOTE_IN_ROBOT));      }
      else if(!(intakeSubsystem.isBeamBreakSensorTriggered()
      ||shooterSubsystem.isBeamBreakSensorTriggered())){
        noteInRobotMsg.ifPresent(RGBMessage::expire);
        noteInRobotMsg = Optional.empty();
      }


        //ready to shoot = red
      if (shooterSubsystem.isReadyToShoot() 
        && pivotSubsystem.isAtTargetDegrees()
        && readyToShootMsg.isEmpty()){
          readyToShootMsg =
            Optional.of(rgbSubsystem.showMessage(Constants.Lights.Colors.RED,
        RGBSubsystem.PatternTypes.PULSE,
        RGBSubsystem.MessagePriority.D_READY_TO_SHOOT));      }
      else if (!(shooterSubsystem.isReadyToShoot() 
        && pivotSubsystem.isAtTargetDegrees())){
        readyToShootMsg.ifPresent(RGBMessage::expire);
        readyToShootMsg = Optional.empty();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
