// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  CANSparkMax flywheelMotor;
  CANSparkMax backspinMotor;

  RelativeEncoder flywheelEncoder;
  RelativeEncoder backspinEncoder;


  /** Creates a new Shooter. */
  public Shooter() {
    flywheelMotor = new CANSparkMax(Constants.flywheelMotorID, MotorType.kBrushless);
    backspinMotor = new CANSparkMax(Constants.backspinMotorID, MotorType.kBrushless);
    flywheelMotor.enableVoltageCompensation(12);
    backspinMotor.enableVoltageCompensation(12);

    flywheelMotor.setSmartCurrentLimit(Constants.flywheelStallCurrentLimit, Constants.flywheelFreeCurrentLimit);
    backspinMotor.setSmartCurrentLimit(Constants.flywheelStallCurrentLimit, Constants.flywheelFreeCurrentLimit);

    flywheelEncoder = flywheelMotor.getEncoder();
    backspinEncoder = backspinMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFlywheelPower(double power){
    flywheelMotor.set(-power);
  }

  public double getFlywheelRPM(){
    return -flywheelEncoder.getVelocity();
  }

  public void setBackspinPower(double power){
    backspinMotor.set(-power);
  }

  public double getBackspinRPM(){
    return -backspinEncoder.getVelocity();
  }

  public RelativeEncoder getFlywheelEncoder(){
    return flywheelMotor.getEncoder();
  }

  public RelativeEncoder getBackspinEncoder(){
    return backspinMotor.getEncoder();
  }

}
