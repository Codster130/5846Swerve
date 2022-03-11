// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Uptake extends SubsystemBase {

  //Front and back belt drivers
  CANSparkMax frontBeltMotor;
  CANSparkMax backBeltMotor;

  //Back and front flywheel feed motors (top of belt uptake)
  CANSparkMax backFeedMotor;
  CANSparkMax frontFeedMotor;

  //Uptake sensors for indexing
  DigitalInput bottomLimitSwitch = new DigitalInput(0);
  
  

  /** Creates a new Uptake. */
  public Uptake() {
    configureUptakeMotors();
    setVoltageCompensation(12);
    setCurrentLimits();

    backBeltMotor.follow(frontBeltMotor);
    backFeedMotor.follow(frontFeedMotor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configureUptakeMotors(){
    //Initialize uptake motors
    frontBeltMotor = new CANSparkMax(Constants.frontBeltMotorID, MotorType.kBrushless);
    backBeltMotor = new CANSparkMax(Constants.backBeltMotorID, MotorType.kBrushless);
    backFeedMotor = new CANSparkMax(Constants.backFeedMotorID, MotorType.kBrushless);
    frontFeedMotor = new CANSparkMax(Constants.frontFeedMotorID, MotorType.kBrushless);
  }

  public void setVoltageCompensation(int voltage){
    frontBeltMotor.enableVoltageCompensation(voltage);

    backBeltMotor.enableVoltageCompensation(voltage);

    backFeedMotor.enableVoltageCompensation(voltage);

    frontFeedMotor.enableVoltageCompensation(voltage);
  }

  public void setCurrentLimits(){
    frontBeltMotor.setSmartCurrentLimit(Constants.beltStallCurrentLimit, Constants.beltFreeCurrentLimit);

    backBeltMotor.setSmartCurrentLimit(Constants.beltStallCurrentLimit, Constants.beltFreeCurrentLimit);

    backFeedMotor.setSmartCurrentLimit(Constants.feedStallCurrentLimit, Constants.feedFreeCurrentLimit);

    frontFeedMotor.setSmartCurrentLimit(Constants.feedStallCurrentLimit, Constants.feedFreeCurrentLimit);
  }

  public void setBeltPower(double power){
    frontBeltMotor.set(power);
  }

  public void setFeedPower(double power){
    frontFeedMotor.set(-power);
  }

  

}
