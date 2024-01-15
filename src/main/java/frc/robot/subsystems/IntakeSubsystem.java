// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import static org.mockito.Mockito.mock;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.Config;
import frc.robot.Constants.Intake;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;

  private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");

  public Modes mode;

  public enum Modes{
    Intake,
    Outtake,
    Hold
  }


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.Intake.INTAKE_MOTOR_PORT);

    intakeMotor.setNeutralMode(NeutralModeValue.Coast);

    intakeMotor.clearStickyFaults();

    //Mode to tell the motor what speed to go at
    mode = Modes.Hold;//default to hold
    
    if(Config.SHOW_SHUFFLEBOARD_DEBUG_DATA){
      tab.add("intake power", intakeMotor.getMotorVoltage().asSupplier());
    }

  }

  @Override
  public void periodic() {
    
    SetIntakeMotorSpeed();

  }

  public void SetIntakeMotorSpeed(){//using the current mode, set the motor speed
    switch(mode){
      case Intake:
        intakeMotor.set(Constants.Intake.INTAKE_MOTOR_SPEED);
        break;
      case Outtake:
        intakeMotor.set(Constants.Intake.OUTTAKE_MOTOR_SPEED);
        break;
      case Hold:
        intakeMotor.set(Constants.Intake.HOLD_MOTOR_SPEED);
        break;
    }
  }
}
