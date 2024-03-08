// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  private RGBMessage noteInRobotMessage;
  private RGBMessage readyToShootMessage;
  private RGBMessage twoNoteMessage;
  private boolean pastBeamBreak;
  private boolean pastTwoNote;
  private boolean pastReadyToShoot;

  

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
            //two note = yellow

      if (shooterSubsystem.isBeamBreakSensorTriggered() && intakeSubsystem.isBeamBreakSensorTriggered()){
        twoNoteMessage = rgbSubsystem.showMessage(Constants.Lights.Colors.YELLOW,
        RGBSubsystem.PatternTypes.PULSE,
        RGBSubsystem.MessagePriority.C_TWO_NOTE_WARNING);
        pastTwoNote = true;
      }
      else if (pastTwoNote){
        twoNoteMessage.expire();
        pastTwoNote = false;
      }
      
    
        //serializer = blue
      if (intakeSubsystem.isBeamBreakSensorTriggered()||shooterSubsystem.isBeamBreakSensorTriggered()){ /*|| shooterSubsystem.isBeamBreakSensorTriggered()*/
        noteInRobotMessage = rgbSubsystem.showMessage(Constants.Lights.Colors.BLUE,
        RGBSubsystem.PatternTypes.PULSE,
        RGBSubsystem.MessagePriority.D_READY_TO_SHOOT);
        pastBeamBreak = true;
      }
      else if (pastBeamBreak){
        noteInRobotMessage.expire();
        pastBeamBreak = false;
      }


        //ready to shoot = red
      if (shooterSubsystem.isReadyToShoot() 
        && pivotSubsystem.isAtTargetDegrees()){
          readyToShootMessage = rgbSubsystem.showMessage(Constants.Lights.Colors.RED,
                    RGBSubsystem.PatternTypes.PULSE,
                    RGBSubsystem.MessagePriority.D_READY_TO_SHOOT);
        }
      else if (pastReadyToShoot){
        readyToShootMessage.expire();
        pastReadyToShoot = false;
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
