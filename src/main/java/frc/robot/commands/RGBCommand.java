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
  

  /** Creates a new RGBCommand. */
  
  public RGBCommand(ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      RGBSubsystem rgbSubsystem,
      PivotSubsystem pivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    // new Trigger(
    //     () -> shooterSubsystem.isBeamBreakSensorTriggered() && intakeSubsystem.isBeamBreakSensorTriggered())
    //     .onTrue( new InstantCommand(() -> {
    //     twoNoteMessage = rgbSubsystem.showMessage(Constants.Lights.Colors.YELLOW,
    //                 RGBSubsystem.PatternTypes.PULSE,
    //                 RGBSubsystem.MessagePriority.C_TWO_NOTE_WARNING);
    //     }, rgbSubsystem))
    //     .onFalse( new InstantCommand(() -> twoNoteMessage.expire()));
    
        //serializer = blue
      if (intakeSubsystem.isBeamBreakSensorTriggered()){ /*|| shooterSubsystem.isBeamBreakSensorTriggered()*/
        noteInRobotMessage = rgbSubsystem.showMessage(/*intakeSubsystem.isBeamBreakSensorTriggered()*/
        Constants.Lights.Colors.PURPLE,
        //: Constants.Lights.Colors.BLUE,
        RGBSubsystem.PatternTypes.PULSE,
        RGBSubsystem.MessagePriority.F_NOTE_IN_ROBOT);
      }
      else {
        noteInRobotMessage.expire();
      }

        //ready to shoot = red
      if (shooterSubsystem.isReadyToShoot() 
        && pivotSubsystem.isAtTargetDegrees()){
          readyToShootMessage = rgbSubsystem.showMessage(Constants.Lights.Colors.RED,
                    RGBSubsystem.PatternTypes.PULSE,
                    RGBSubsystem.MessagePriority.D_READY_TO_SHOOT);
        }
      else{
        readyToShootMessage.expire();
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
