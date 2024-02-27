// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private CANSparkMax m_climbLeft;
  private CANSparkMax m_climbRight;
  private SparkPIDController climbLeftPIDController;
  private SparkPIDController climbRightPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private DutyCycleEncoder climbLeftEncoder;
  private DutyCycleEncoder climbRightEncoder;
  private double climbOffset;
  private Boolean encodersAreReset;
  private double count;

  public Climb() {
    m_climbLeft = new CANSparkMax(Constants.ClimbConstants.climbMotorLeftID, MotorType.kBrushless);
    m_climbRight = new CANSparkMax(Constants.ClimbConstants.climbMotorRightID, MotorType.kBrushless);
    climbLeftEncoder = new DutyCycleEncoder(Constants.ClimbConstants.climbLeftEncoderID);
    climbRightEncoder = new DutyCycleEncoder(Constants.ClimbConstants.climbRightEncoderID);
    climbOffset = Constants.ClimbConstants.climbOffset;
  }

  public void configMotors(){

     //Neo 550 Conifg
     m_climbLeft.restoreFactoryDefaults();
     m_climbLeft.setSmartCurrentLimit(20);
     m_climbLeft.setInverted(false);
     m_climbLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);

     m_climbRight.restoreFactoryDefaults();
     m_climbRight.setSmartCurrentLimit(20);
     m_climbRight.setInverted(false);
     m_climbRight.setIdleMode(CANSparkMax.IdleMode.kCoast);

    //Get PID Controller and Encoder
    climbLeftPIDController = m_climbLeft.getPIDController();
    climbRightPIDController = m_climbRight.getPIDController();

    //PID Constants
    kP = Constants.ClimbConstants.kP; 
    kI = Constants.ClimbConstants.kI;
    kD = Constants.ClimbConstants.kD;
    kFF = Constants.ClimbConstants.kFF; 
    kMaxOutput = Constants.ClimbConstants.kMaxOutput; 
    kMinOutput = Constants.ClimbConstants.kMinOutput;

  }


//Methods
  public void resetMotorToAbsolute(){
    double offsetClimbLeftPos = climbLeftEncoder.getAbsolutePosition() - climbOffset;
    double offsetClimbRightPos = climbRightEncoder.getAbsolutePosition() - climbOffset;
    if (climbLeftEncoder.isConnected() && climbRightEncoder.isConnected()) { 
      m_climbLeft.getEncoder().setPosition(offsetClimbLeftPos);
      m_climbRight.getEncoder().setPosition(offsetClimbRightPos);
      encodersAreReset = true;
    } 
    else {
      encodersAreReset = false;
    }
    
  }

  public void setLeftClimbPos(double setpoint){
    if (encodersAreReset) {
      climbLeftPIDController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }
  }

  public void setRightClimbPos(double setpoint){
    if (encodersAreReset) {
      climbRightPIDController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }
  }

  public double getLeftClimbPos(){
    return m_climbLeft.getEncoder().getPosition();
  }

  public double getRightClimbPos(){
    return m_climbRight.getEncoder().getPosition();
  }
  
  public double getLeftClimbVel(){
    return m_climbLeft.getEncoder().getVelocity();
  }

  public double getRightClimbVel(){
    return m_climbRight.getEncoder().getVelocity();
  }

  public void setBothPos(double setpoint){
    if (encodersAreReset){
      climbLeftPIDController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
      climbRightPIDController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);

    }
  }

  public void stopClimb(){
    m_climbLeft.stopMotor();
    m_climbRight.stopMotor();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!encodersAreReset && count <= 0) {
      resetMotorToAbsolute();
    } else {
      count += 1;
      count %= 5;
    }

    SmartDashboard.putNumber("LeftClimbPos", getLeftClimbPos());
    SmartDashboard.putNumber("RightClimbPos", getRightClimbPos());
    SmartDashboard.putNumber("LeftClimbVel", getLeftClimbVel());
    SmartDashboard.putNumber("RightClimbVel", getRightClimbVel());
  }
}