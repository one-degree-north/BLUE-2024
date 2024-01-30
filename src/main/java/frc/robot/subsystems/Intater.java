// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
 import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intater extends SubsystemBase {
  /** Creates a new Intater. */
  private CANSparkMax m_flywheelLeft;
  private CANSparkMax m_flywheelRight;
  private RelativeEncoder flywheelLeftEncoder;
  private RelativeEncoder flywheelRightEncoder;
  private SparkPIDController flywheelLeftPidController;
  private SparkPIDController flywheelRightPidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  public Intater() {
    m_flywheelLeft = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
    m_flywheelRight = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
  }

  private void configMotors(){

    //NEO 500 Configs
    m_flywheelLeft.restoreFactoryDefaults();
    m_flywheelLeft.setSmartCurrentLimit(20);
    m_flywheelLeft.setInverted(false);
    m_flywheelLeft.setOpenLoopRampRate(0.25);

    m_flywheelRight.restoreFactoryDefaults();
    m_flywheelRight.setSmartCurrentLimit(20);
    m_flywheelRight.setInverted(false);
    m_flywheelRight.setOpenLoopRampRate(0.25);

    //Get PID Controller
    flywheelLeftPidController = m_flywheelLeft.getPIDController();
    flywheelRightPidController = m_flywheelRight.getPIDController();

    //PID Configs
    kP = 0; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0; 
    kMinOutput = 0;

    flywheelLeftPidController.setP(kP);
    flywheelLeftPidController.setI(kI);
    flywheelLeftPidController.setD(kD);
    flywheelLeftPidController.setIZone(kIz);
    flywheelLeftPidController.setFF(kFF);
    flywheelLeftPidController.setOutputRange(kMinOutput, kMaxOutput);

    flywheelRightPidController.setP(kP);
    flywheelRightPidController.setI(kI);
    flywheelRightPidController.setD(kD);
    flywheelRightPidController.setIZone(kIz);
    flywheelRightPidController.setFF(kFF);
    flywheelRightPidController.setOutputRange(kMinOutput, kMaxOutput);

  }

  public void setLeftSpeed(double velocity){
    flywheelLeftPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setRightSpeed(double velocity){
    flywheelRightPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setBothSpeed(double velocity){
    flywheelRightPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    flywheelLeftPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public double getLeftSpeed(){
    return flywheelLeftEncoder.getVelocity();
  }

  public double getRightSpeed(){
    return flywheelRightEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
