// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {  
  /** Creates a new Pivot. */

  private CANSparkMax m_pivotLead;
  private CANSparkMax m_pivotFollow;
  private DutyCycleEncoder pivotEncoder;
  private double pivotOffset;
  private double kP, kI, kD;
  private double maxVel, maxAccel;
  private ProfiledPIDController pivotPIDController;

  public Pivot() {

    m_pivotLead = new CANSparkMax(PivotConstants.pivotLeadID, MotorType.kBrushless);
    m_pivotFollow = new CANSparkMax(PivotConstants.pivotFollowID, MotorType.kBrushless);
    pivotPIDController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVel, maxAccel));
    pivotEncoder = new DutyCycleEncoder(PivotConstants.throughBoreID);
    pivotPIDController.setTolerance(PivotConstants.PivotTolerance);
    configMotors();
    resetMotorToAbsolute();

  }

  private void configMotors(){
    pivotOffset = 0.717;

    //Neo Conifg
    m_pivotLead.restoreFactoryDefaults();
    m_pivotLead.setSmartCurrentLimit(20);
    m_pivotLead.setInverted(false);
    m_pivotLead.setOpenLoopRampRate(Constants.PivotConstants.PivotOpenLoopRampRate);
    m_pivotLead.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_pivotLead.getEncoder().setPositionConversionFactor(1.0/PivotConstants.pivotGearRatio);
    m_pivotLead.getEncoder().setVelocityConversionFactor(
      (1.0/PivotConstants.pivotGearRatio)*(1.0/60.0));

    m_pivotFollow.restoreFactoryDefaults();
    m_pivotFollow.setSmartCurrentLimit(20);
    m_pivotFollow.setOpenLoopRampRate(Constants.PivotConstants.PivotOpenLoopRampRate);
    m_pivotFollow.setIdleMode(CANSparkMax.IdleMode.kCoast);
    

    kP = Constants.PivotConstants.kP;
    kI = Constants.PivotConstants.kI;
    kD = Constants.PivotConstants.kD;

    m_pivotFollow.follow(m_pivotLead, true);

  }

  //Methods
  public void resetMotorToAbsolute(){
    double offsetPivotPos = pivotEncoder.getAbsolutePosition() - pivotOffset;
      m_pivotLead.getEncoder().setPosition(offsetPivotPos);
  }

  public void setPivotPos(double setpoint){
    pivotPIDController.setGoal(setpoint);
  }

  public double getPivotPos(){
    return m_pivotLead.getEncoder().getPosition();
  }

  public double getPivotRPS(){
    return m_pivotLead.getEncoder().getVelocity();
  }

  public void stopAll(){
    m_pivotLead.stopMotor();
    m_pivotFollow.stopMotor();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Pivot RPS", getPivotRPS());
    SmartDashboard.putNumber("Pivot Rotations", getPivotPos());  
    SmartDashboard.putNumber("Absolute Encoder Rotations", pivotEncoder.getAbsolutePosition());

    m_pivotLead.setVoltage(
      pivotPIDController.calculate(getPivotPos())
    );
  }
  
}
