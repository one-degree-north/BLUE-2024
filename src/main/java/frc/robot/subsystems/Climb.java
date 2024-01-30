// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

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
    m_climbLeft = new CANSparkMax(0, MotorType.kBrushless);
    m_climbRight = new CANSparkMax(0, MotorType.kBrushless);
    climbLeftEncoder = new DutyCycleEncoder(0);
    climbRightEncoder = new DutyCycleEncoder(0);
    climbOffset = 0;
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
    kP = 0; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0; 
    kMinOutput = 0;

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

  public void setLeftClimb(double speed){
    if (encodersAreReset) {
      climbLeftPIDController.setReference(speed, CANSparkMax.ControlType.kSmartVelocity);
    }
  }

  public void setRightClimb(double speed){
    if (encodersAreReset) {
      climbRightPIDController.setReference(speed, CANSparkMax.ControlType.kSmartVelocity);
    }
  }

  public double getLeftClimbPos(){
    return m_climbLeft.getEncoder().getPosition();
  }

  public double getRightClimbPos(){
    return m_climbRight.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (encodersAreReset && count <= 0) {
      resetMotorToAbsolute();
    } else {
      count += 1;
      count %= 5;
    }
  }
}
