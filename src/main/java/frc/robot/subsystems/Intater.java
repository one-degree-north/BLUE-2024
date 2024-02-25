// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants.IntaterConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private DigitalInput intakeSensor;
  
  public Intater() {
    m_flywheelLeft = new CANSparkMax(IntaterConstants.flywheelLeftID, CANSparkMax.MotorType.kBrushless);
    m_flywheelRight = new CANSparkMax(IntaterConstants.flywheelRightID, CANSparkMax.MotorType.kBrushless);
    m_intake = new CANSparkMax(IntaterConstants.intakeID, CANSparkMax.MotorType.kBrushless);
    intakeSensor = new DigitalInput(IntaterConstants.intakeSensorID);
    configMotors();
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
    m_intake.setIdleMode(IdleMode.kBrake);

    //Get PID Controller
    flywheelLeftPIDController = m_flywheelLeft.getPIDController();
    flywheelRightPIDController = m_flywheelRight.getPIDController();
    intakePIDController = m_intake.getPIDController();

    flywheelLeftEncoder = m_flywheelLeft.getEncoder();
    flywheelRightEncoder = m_flywheelRight.getEncoder();
    intakeEncoder = m_intake.getEncoder();

    //PID Configs
    flywheelLeftPIDController.setP(IntaterConstants.FlywheelLeftkP);
    flywheelLeftPIDController.setI(IntaterConstants.FlywheelLeftkI);
    flywheelLeftPIDController.setD(IntaterConstants.FlywheelLeftkD);
    flywheelLeftPIDController.setFF(IntaterConstants.FlywheelLeftkFF);
    flywheelLeftPIDController.setOutputRange(IntaterConstants.FlywheelLeftkMinOutput, IntaterConstants.FlywheelLeftkMaxOutput);

    flywheelRightPIDController.setP(IntaterConstants.FlywheelRightkP);
    flywheelRightPIDController.setI(IntaterConstants.FlywheelRightkI);
    flywheelRightPIDController.setD(IntaterConstants.FlywheelRightkD);
    flywheelRightPIDController.setFF(IntaterConstants.FlywheelRightkFF);
    flywheelRightPIDController.setOutputRange(IntaterConstants.FlywheelRightkMinOutput, IntaterConstants.FlywheelRightkMaxOutput);

    intakePIDController.setP(IntaterConstants.IntakekP);
    intakePIDController.setI(IntaterConstants.IntakekI);
    intakePIDController.setD(IntaterConstants.IntakekD);
    intakePIDController.setFF(IntaterConstants.IntakekFF);
    intakePIDController.setOutputRange(IntaterConstants.IntakekMinOutput, IntaterConstants.IntakekMaxOutput);
  }

  //Methods

  public void stopAll() {
    stopIntake();
    stopShooter();
  }

  public void stopIntake() {
    setIntakeSpeed(0);
  }

  public void stopShooter() {
    setLeftSpeed(0);
    setRightSpeed(0);
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

  public double getLeftSpeedRPS(){
    return flywheelLeftEncoder.getVelocity();
  }

  public double getRightSpeedRPS(){
    return flywheelRightEncoder.getVelocity();
  }

  public double getIntakeSpeedRPS(){
    return intakeEncoder.getVelocity();
  }

  public double getIntakePercentSpeed(){
    return m_intake.get();
  }

  public double getLeftPercentSpeed(){
    return m_flywheelLeft.get();
  }

  public double getRightPercentSpeed(){
    return m_flywheelRight.get();
  }

  public boolean isNoteIntaked() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Intake RPS", getIntakeSpeedRPS());
    SmartDashboard.putNumber("Left Shooter RPS", getLeftSpeedRPS());
    SmartDashboard.putNumber("Right Shooter RPS", getRightSpeedRPS());
    SmartDashboard.putBoolean("NoteIntaked", isNoteIntaked());
  }
  
}
