// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intater extends SubsystemBase {
  /** Creates a new Intater. */
  private CANSparkMax m_flywheelLeft;
  private CANSparkMax m_flywheelRight;
  private CANSparkMax m_intake;
  private RelativeEncoder flywheelLeftEncoder;
  private RelativeEncoder flywheelRightEncoder;
  private RelativeEncoder intakeEncoder;
  private SparkPIDController flywheelLeftPIDController;
  private SparkPIDController flywheelRightPIDController;
  private SparkPIDController intakePIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  
  public Intater() {
    m_flywheelLeft = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);
    m_flywheelRight = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
    m_intake = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
  }

  private void configMotors(){

    //NEO 500 Configs
    m_flywheelLeft.restoreFactoryDefaults();
    m_flywheelLeft.setSmartCurrentLimit(20);
    m_flywheelLeft.setInverted(false);
    m_flywheelLeft.setIdleMode(IdleMode.kCoast);

    m_flywheelRight.restoreFactoryDefaults();
    m_flywheelRight.setSmartCurrentLimit(20);
    m_flywheelRight.setInverted(false);
    m_flywheelRight.setIdleMode(IdleMode.kCoast);
    

    m_intake.restoreFactoryDefaults();
    m_intake.setSmartCurrentLimit(20);
    m_intake.setInverted(false);
    m_intake.setIdleMode(IdleMode.kCoast);

    //Get PID Controller
    flywheelLeftPIDController = m_flywheelLeft.getPIDController();
    flywheelRightPIDController = m_flywheelRight.getPIDController();
    intakePIDController = m_intake.getPIDController();

    flywheelLeftEncoder = m_flywheelLeft.getEncoder();
    flywheelRightEncoder = m_flywheelRight.getEncoder();
    intakeEncoder = m_intake.getEncoder();

    //PID Configs
    kP = 0; 
    kI = 0;
    kD = 0; 
    kFF = 0; 
    kMaxOutput = 0; 
    kMinOutput = 0;

    flywheelLeftPIDController.setP(kP);
    flywheelLeftPIDController.setI(kI);
    flywheelLeftPIDController.setD(kD);
    flywheelLeftPIDController.setFF(kFF);
    flywheelLeftPIDController.setOutputRange(kMinOutput, kMaxOutput);

    flywheelRightPIDController.setP(kP);
    flywheelRightPIDController.setI(kI);
    flywheelRightPIDController.setD(kD);
    flywheelRightPIDController.setFF(kFF);
    flywheelRightPIDController.setOutputRange(kMinOutput, kMaxOutput);

    intakePIDController.setP(kP);
    intakePIDController.setI(kI);
    intakePIDController.setD(kD);
    intakePIDController.setFF(kFF);
    intakePIDController.setOutputRange(kMinOutput, kMaxOutput);

  }

  public void setLeftSpeed(double velocity){
    flywheelLeftPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setRightSpeed(double velocity){
    flywheelRightPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setBothSpeed(double velocity){
    flywheelRightPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    flywheelLeftPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public void setIntakeSpeed(double velocity){
    intakePIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public double getLeftSpeed(){
    return flywheelLeftEncoder.getVelocity();
  }

  public double getRightSpeed(){
    return flywheelRightEncoder.getVelocity();
  }

  public double getIntakeSpeed(){
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
