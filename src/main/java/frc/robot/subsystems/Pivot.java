// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */

  private CANSparkMax m_pivotLead;
  private CANSparkMax m_pivotFollow;
  private SparkPIDController pivotPIDController;
  private RelativeEncoder pivotEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public Pivot() {

    m_pivotLead = new CANSparkMax(0, MotorType.kBrushless);
    m_pivotFollow = new CANSparkMax(0, MotorType.kBrushless);

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
    pivotEncoder = m_pivotLead.getEncoder();

    //PID Controller Config
    kP = 0; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0; 
    kMinOutput = 0;

    pivotPIDController.setP(kP);
    pivotPIDController.setI(kI);
    pivotPIDController.setD(kD);
    pivotPIDController.setIZone(kIz);
    pivotPIDController.setFF(kFF);
    pivotPIDController.setOutputRange(kMinOutput, kMaxOutput);

  }

  public void setPivot(double setpoint){
    pivotPIDController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  public double getMotorPos(){
    return pivotEncoder.getPosition();
  }

  public double getMotorVel(){
    return pivotEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
