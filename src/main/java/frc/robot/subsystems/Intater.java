// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants.IntaterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intater extends SubsystemBase {
  /** Creates a new Intater. */
  private CANSparkMax m_flywheelLeft;
  private CANSparkMax m_flywheelRight;
  private CANSparkMax m_intake;
  private RelativeEncoder flywheelLeftEncoder;
  private RelativeEncoder flywheelRightEncoder;
  private SparkPIDController flywheelLeftPIDController;
  private SparkPIDController flywheelRightPIDController;
  
  public Intater() {
    m_flywheelLeft = new CANSparkMax(IntaterConstants.flywheelLeftID, CANSparkMax.MotorType.kBrushless);
    m_flywheelRight = new CANSparkMax(IntaterConstants.flywheelRightID, CANSparkMax.MotorType.kBrushless);
    m_intake = new CANSparkMax(IntaterConstants.intakeID, CANSparkMax.MotorType.kBrushless);    
    configMotors();
  }

  private void configMotors(){

    //NEO 500 Configs
    m_flywheelLeft.restoreFactoryDefaults();
    m_flywheelLeft.setSmartCurrentLimit(40);
    m_flywheelLeft.setInverted(true);
    m_flywheelLeft.setIdleMode(IdleMode.kCoast);

    m_flywheelRight.restoreFactoryDefaults();
    m_flywheelRight.setSmartCurrentLimit(40);
    m_flywheelRight.setInverted(false);
    m_flywheelRight.setIdleMode(IdleMode.kCoast);
    

    m_intake.restoreFactoryDefaults();
    m_intake.setSmartCurrentLimit(60);
    m_intake.setInverted(true);
    m_intake.setIdleMode(IdleMode.kCoast);

    //Get PID Controller
    flywheelLeftPIDController = m_flywheelLeft.getPIDController();
    flywheelRightPIDController = m_flywheelRight.getPIDController();

    flywheelLeftEncoder = m_flywheelLeft.getEncoder();
    flywheelRightEncoder = m_flywheelRight.getEncoder();

    flywheelLeftEncoder.setPositionConversionFactor(1.0/IntaterConstants.flywheelGearRatio);
    flywheelLeftEncoder.setVelocityConversionFactor((1.0/IntaterConstants.flywheelGearRatio) * (1.0/60.0) );
    flywheelRightEncoder.setPositionConversionFactor(1.0/IntaterConstants.flywheelGearRatio);
    flywheelRightEncoder.setVelocityConversionFactor((1.0/IntaterConstants.flywheelGearRatio) * (1.0/60.0) );

    //PID Configs
    flywheelLeftPIDController.setP(IntaterConstants.FlywheelLeftkP);
    flywheelLeftPIDController.setI(IntaterConstants.FlywheelLeftkI);
    flywheelLeftPIDController.setD(IntaterConstants.FlywheelLeftkD);
    flywheelLeftPIDController.setFF(IntaterConstants.FlywheelLeftkFF);

    flywheelRightPIDController.setP(IntaterConstants.FlywheelRightkP);
    flywheelRightPIDController.setI(IntaterConstants.FlywheelRightkI);
    flywheelRightPIDController.setD(IntaterConstants.FlywheelRightkD);
    flywheelRightPIDController.setFF(IntaterConstants.FlywheelRightkFF);

    flywheelLeftPIDController.setFeedbackDevice(flywheelLeftEncoder);
    flywheelRightPIDController.setFeedbackDevice(flywheelRightEncoder);

    m_flywheelLeft.enableVoltageCompensation(12);
    m_flywheelRight.enableVoltageCompensation(12);

    m_flywheelLeft.burnFlash();
    m_flywheelRight.burnFlash();
  }

  //Methods

  public void stopAll() {
    stopIntake();
    stopShooter();
  }

  public void stopIntake() {
    setIntakeSpeedDutyCycle(0);
  }

  public void stopShooter() {
    
    setBothSpeedVoltage(0);
  }

  public void setLeftSpeedRPS(double velocity){
    flywheelLeftPIDController.setReference(velocity, ControlType.kVelocity);
  }

  public void setRightSpeedRPS(double velocity){
    flywheelRightPIDController.setReference(velocity, ControlType.kVelocity);
  }

  public void setBothSpeedRPS(double velocity){
    setLeftSpeedRPS(velocity);
    setRightSpeedRPS(velocity);
  }

  public void setBothSpeedVoltage(double volts) {
    m_flywheelLeft.setVoltage(volts);
    m_flywheelRight.setVoltage(volts);
  }

  public void setIntakeSpeedDutyCycle(double velocity){
    m_intake.set(velocity);
  }

  public double getLeftSpeedRPS(){
    return flywheelLeftEncoder.getVelocity();
  }

  public double getRightSpeedRPS(){
    return flywheelRightEncoder.getVelocity();
  }

  public double getIntakePercentSpeed(){
    return m_intake.get();
  }

  public double getLeftPercentSpeed(){
    return (m_flywheelLeft.get());
  }

  public double getRightPercentSpeed(){
    return (m_flywheelRight.get());
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Intake RPS", getIntakePercentSpeed());
    SmartDashboard.putNumber("Left Shooter RPS", getLeftSpeedRPS());
    SmartDashboard.putNumber("Right Shooter RPS", getRightSpeedRPS());;
  }
  
}
