// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntaterConstants;
import frc.robot.subsystems.Intater;

public class IntaterCommand extends Command {
  /** Creates a new Intater. */

  private IntaterMode m_mode;
  private Intater s_Intater;
  private Command m_commandToRun;

  public IntaterCommand(IntaterMode mode, Intater intater) {
    this.s_Intater = intater;
    addRequirements(s_Intater);

    this.m_mode = mode;
  }
  public IntaterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (m_mode) {
      case SPEAKERSHOOT:
        m_commandToRun = 
        new InstantCommand(()-> s_Intater.setIntakeSpeedDutyCycle(IntaterConstants.OuttakeVelDutyCycle))
        .andThen(Commands.waitSeconds(0.05))
        .andThen(new InstantCommand(() -> s_Intater.stopIntake()))
        .andThen(new InstantCommand(() -> s_Intater.setBothSpeedRPS(Constants.IntaterConstants.SpeakerVelRPS)))
        .andThen(Commands.waitSeconds(3.5))
        .andThen(new InstantCommand(() -> s_Intater.setIntakeSpeedDutyCycle(IntaterConstants.IntakeFeedDutyCycle)))
        .andThen(Commands.waitUntil(() -> false));
        break;

      case AMPINTAKE:
        m_commandToRun = new InstantCommand(() -> s_Intater.setIntakeSpeedDutyCycle(Constants.IntaterConstants.IntakeVelDutyCycle))
        .alongWith(new InstantCommand(() -> s_Intater.setBothSpeedRPS(50)))
        .alongWith(Commands.waitUntil(() -> false));
        break;
      case INTAKE:
        m_commandToRun = new InstantCommand(() -> s_Intater.setIntakeSpeedDutyCycle(Constants.IntaterConstants.IntakeVelDutyCycle))
        .alongWith(new InstantCommand(() -> s_Intater.setBothSpeedRPS(-IntaterConstants.ShooterCounteractingRPS)))
        .alongWith(Commands.waitUntil(() -> false));
        break;

      case OUTTAKE:
        m_commandToRun = new InstantCommand(() -> s_Intater.setIntakeSpeedDutyCycle(Constants.IntaterConstants.OuttakeVelDutyCycle))
        .alongWith(new InstantCommand(() -> s_Intater.setBothSpeedRPS(-IntaterConstants.ShooterCounteractingRPS)))
        .alongWith(Commands.waitUntil(() -> false));
        break;

      case AMP:
        m_commandToRun = 
        new InstantCommand(() -> s_Intater.setIntakeSpeedDutyCycle(IntaterConstants.OuttakeVelDutyCycle))
        .alongWith(new InstantCommand(() -> s_Intater.setBothSpeedRPS(Constants.IntaterConstants.ShooterCounteractingRPS)))
        .alongWith(Commands.waitSeconds(0.2))
        .andThen(new InstantCommand(() -> s_Intater.setIntakeSpeedDutyCycle(Constants.IntaterConstants.OuttakeVelDutyCycle)))
        .alongWith(new InstantCommand(() -> s_Intater.setBothSpeedRPS(-IntaterConstants.ShooterCounteractingRPS)))
        .alongWith(Commands.waitUntil(() -> false));
        break;
      
      case STOP:
        m_commandToRun = new InstantCommand(() -> s_Intater.stopAll());
      
      case INTAKEANDSHOOT:
        m_commandToRun = new InstantCommand(() -> s_Intater.setIntakeSpeedDutyCycle(IntaterConstants.IntakeVelDutyCycle))
        .alongWith(new InstantCommand(() -> s_Intater.setBothSpeedRPS(IntaterConstants.SpeakerVelRPS)))
        .alongWith(Commands.waitUntil(() -> false));
      default:
        break;

    }

    m_commandToRun.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_commandToRun.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intater.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandToRun.isFinished();
  }

  public enum IntaterMode {
    SPEAKERSHOOT, INTAKE, STOP, OUTTAKE, AMPINTAKE, LEVEL, INTAKEANDSHOOT, AMP;
  }
}
