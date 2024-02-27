// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {  
  /** Creates a new Pivot. */

  private CANSparkMax m_pivotLead;
  private CANSparkMax m_pivotFollow;
  private SparkPIDController pivotPIDController;
  private AbsoluteEncoder pivotEncoder;
  private double pivotOffset;
  private Boolean encodersAreReset;

  public Pivot() {

    m_pivotLead = new CANSparkMax(PivotConstants.pivotLeadID, MotorType.kBrushless);
    m_pivotFollow = new CANSparkMax(PivotConstants.pivotFollowID, MotorType.kBrushless);
    configMotors();

  }

  private void configMotors(){

    m_pivotFollow.follow(m_pivotLead);

    //Neo 550 Conifg
    m_pivotLead.restoreFactoryDefaults();
    m_pivotLead.setSmartCurrentLimit(20);
    m_pivotLead.setInverted(false);
    m_pivotLead.setOpenLoopRampRate(0);
    m_pivotLead.setIdleMode(CANSparkMax.IdleMode.kCoast);

    //Get PID Controller and Encoder
    pivotPIDController = m_pivotLead.getPIDController();
    pivotEncoder.setPositionConversionFactor(PivotConstants.PosConversionFactor);
    pivotEncoder.setVelocityConversionFactor(PivotConstants.VelConversionFactor);

    //PID Controller Config
    pivotPIDController.setP(PivotConstants.kP);
    pivotPIDController.setI(PivotConstants.kI);
    pivotPIDController.setD(PivotConstants.kD);
    pivotPIDController.setFF(PivotConstants.kFF);
    pivotPIDController.setOutputRange(PivotConstants.kMinOutput, PivotConstants.kMaxOutput);

  }

  //Methods
  public void resetMotorToAbsolute(){
    double offsetPivotPos = pivotEncoder.getPosition() - pivotOffset;
    if (encodersAreReset == false) { 
      m_pivotLead.getEncoder().setPosition(offsetPivotPos);
      encodersAreReset = true;
    } 

    else {
      encodersAreReset = false;
    }
  }

  public void setPivotPos(double setpoint){
    pivotPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  public void zeroPivot(){
    m_pivotLead.getEncoder().setPosition(0);
  }

  public double getPivotPos(){
    return pivotEncoder.getPosition();
  }

  public double getPivotRPS(){
    return pivotEncoder.getVelocity();
  }

  public void stopAll(){
    m_pivotLead.stopMotor();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("PivotRPS", getPivotPos());
    SmartDashboard.putNumber("PivotPos", getPivotRPS());  
  }
  
}
