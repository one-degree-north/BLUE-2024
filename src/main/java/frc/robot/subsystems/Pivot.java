// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {  
  /** Creates a new Pivot. */

  private CANSparkMax m_pivotLead;
  private CANSparkMax m_pivotFollow;
  private AbsoluteEncoder pivotEncoder;
  private double pivotOffset;
  private Boolean encodersAreReset;
  private double kP, kI, kD;
  private double kA, kG, kS, kV;
  private ProfiledPIDController pivotPIDController;
  private ArmFeedforward pivotFFController;

  public Pivot() {

    m_pivotLead = new CANSparkMax(PivotConstants.pivotLeadID, MotorType.kBrushless);
    m_pivotFollow = new CANSparkMax(PivotConstants.pivotFollowID, MotorType.kBrushless);
    pivotPIDController = new ProfiledPIDController(kP, kI, kD, null);

    configMotors();

  }

  private void configMotors(){

    m_pivotFollow.follow(m_pivotLead);

    //Neo 550 Conifg
    m_pivotLead.restoreFactoryDefaults();
    m_pivotLead.setSmartCurrentLimit(20);
    m_pivotLead.setInverted(false);
    m_pivotLead.setOpenLoopRampRate(Constants.PivotConstants.PivotOpenLoopRampRate);
    m_pivotLead.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_pivotFollow.restoreFactoryDefaults();
    m_pivotFollow.setSmartCurrentLimit(20);
    m_pivotFollow.setInverted(false);
    m_pivotFollow.setOpenLoopRampRate(Constants.PivotConstants.PivotOpenLoopRampRate);
    m_pivotFollow.setIdleMode(CANSparkMax.IdleMode.kCoast);

    //Get PID Values
    kA = Constants.PivotConstants.kA; 
    kV = Constants.PivotConstants.kV;
    kS = Constants.PivotConstants.kS;
    kG = Constants.PivotConstants.kG;

    kP = Constants.PivotConstants.kP;
    kI = Constants.PivotConstants.kI;
    kD = Constants.PivotConstants.kD;

    //PID Controller Config
    pivotFFController = new ArmFeedforward(kS, kG, kV, kA);
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
    pivotPIDController.setGoal(setpoint);
  }

  public void levelPivot(){
    pivotPIDController.setGoal(PivotConstants.LevelPosition);
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

    SmartDashboard.putNumber("PivotRPS", getPivotPos());
    SmartDashboard.putNumber("PivotPos", getPivotRPS());  

    m_pivotLead.setVoltage(
      pivotPIDController.calculate(getPivotPos())
          +
          pivotFFController.calculate(
          pivotPIDController.getSetpoint().position,
          pivotPIDController.getSetpoint().velocity
          )

    );
  }
  
}
