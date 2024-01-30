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

  private CANSparkMax m_pivot;
  private SparkPIDController m_pivotPidController;
  private RelativeEncoder m_pivotEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public Pivot() {

    m_pivot = new CANSparkMax(0, MotorType.kBrushless);

  }

  private void configMotors(){

    //Neo 550 Conifg
    m_pivot.restoreFactoryDefaults();
    m_pivot.setSmartCurrentLimit(20);
    m_pivot.setInverted(false);
    m_pivot.setOpenLoopRampRate(0);
    m_pivot.setIdleMode(CANSparkMax.IdleMode.kCoast);

    //Get PID Controller and Encoder
    m_pivotPidController = m_pivot.getPIDController();
    m_pivotEncoder = m_pivot.getEncoder();

    //PID Controller Config
    kP = 0; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0; 
    kMinOutput = 0;

    m_pivotPidController.setP(kP);
    m_pivotPidController.setI(kI);
    m_pivotPidController.setD(kD);
    m_pivotPidController.setIZone(kIz);
    m_pivotPidController.setFF(kFF);
    m_pivotPidController.setOutputRange(kMinOutput, kMaxOutput);


  }

  public void setPivot(double setpoint){
    m_pivotPidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  public double getMotorPos(){
    return m_pivotEncoder.getPosition();
  }

  public double getMotorVel(){
    return m_pivotEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
