// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends Command {
  /** Creates a new ClimbCommand. */
  private ClimbMode m_mode;
  private Climb s_Climb;
  private Command m_commandToRun;

  public ClimbCommand(ClimbMode mode, Climb climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Climb = climb;
    addRequirements(s_Climb);
  

    this.m_mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_mode) {
      case CLIMB_UP:
      m_commandToRun = new InstantCommand(() -> s_Climb.setBothPos(ClimbConstants.ClimbUpPosition))
      .alongWith(Commands.waitUntil(() -> false));
        break;

      case CLIMB_DOWN:
      m_commandToRun = new InstantCommand(() -> s_Climb.setBothPos(ClimbConstants.ClimbDownPosition))
      .alongWith(Commands.waitUntil(() -> false));
        break;

      case STOP_CLIMB:
        m_commandToRun = new InstantCommand(() -> s_Climb.stopClimb());
        break;
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
      s_Climb.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_commandToRun.isFinished();
  }

  public enum ClimbMode {
    CLIMB_UP, CLIMB_DOWN, STOP_CLIMB
  }
}
