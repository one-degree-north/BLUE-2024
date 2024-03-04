// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

public class PivotCommand extends Command {
  /** Creates a new PivotCommand. */

  private PivotMode m_mode;
  private Pivot s_Pivot;
  private Command m_commandToRun;
  private boolean m_stopWhenFinished;
  
  public PivotCommand(PivotMode mode, Pivot pivot, boolean stopWhenFinished) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Pivot = pivot;
    addRequirements(s_Pivot);
  

    this.m_mode = mode;
    this.m_stopWhenFinished = stopWhenFinished;
  }
  
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (m_mode) {
      case SPEAKER:
        m_commandToRun = 
        new InstantCommand(() -> s_Pivot.setPivotPos(PivotConstants.SpeakerPosition));
        break;  

      case AMP:
        m_commandToRun = 
        new InstantCommand(() -> s_Pivot.setPivotPos(PivotConstants.AmpPosition));
        break;

      case GROUND_INTAKE:
        m_commandToRun = 
        new InstantCommand(() -> s_Pivot.setPivotPos(PivotConstants.GroundIntakePosition));
        break;
      
      case SOURCE_INTAKE:
        m_commandToRun = 
        new InstantCommand(() -> s_Pivot.setPivotPos(PivotConstants.SourceIntakePosition));
        break;

      case STOP:
        m_commandToRun = 
        new InstantCommand(() -> s_Pivot.stopAll());

      case LEVEL:
        m_commandToRun = 
        new InstantCommand(() -> s_Pivot.setPivotPos(PivotConstants.LevelPosition));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_commandToRun.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_stopWhenFinished)
      s_Pivot.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandToRun.isFinished();
  }

  public enum PivotMode {
    SPEAKER, AMP, GROUND_INTAKE, SOURCE_INTAKE, STOP, LEVEL;
  }
}
