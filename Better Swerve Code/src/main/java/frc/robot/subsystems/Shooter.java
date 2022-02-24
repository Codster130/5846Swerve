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
  CANSparkMax turretMotor;
  RelativeEncoder flywheelEncoder;
  RelativeEncoder backspinEncoder;
  RelativeEncoder turretEncoder;
  SparkMaxPIDController turretPID;

  /** Creates a new Shooter. */
  public Shooter() {
    flywheelMotor = new CANSparkMax(Constants.flywheelMotorID, MotorType.kBrushless);
    backspinMotor = new CANSparkMax(Constants.backspinMotorID, MotorType.kBrushless);
    turretMotor = new CANSparkMax(Constants.turretMotorID, MotorType.kBrushless);
    turretPID = turretMotor.getPIDController();
    flywheelMotor.enableVoltageCompensation(12);
    backspinMotor.enableVoltageCompensation(12);
    //turretMotor.enableVoltageCompensation(12);
    flywheelEncoder = flywheelMotor.getEncoder();
    backspinEncoder = backspinMotor.getEncoder();
    turretEncoder = turretMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTurretPower(double power){
    turretMotor.set(power);
  }

  public double getTurretPosition(){
    return turretEncoder.getPosition();
  }

  public void setFlywheelPower(double power){
    flywheelMotor.set(power);
  }

  public double getFlywheelRPM(){
    return flywheelEncoder.getVelocity();
  }

  public void setBackspinPower(double power){
    backspinMotor.set(power);
  }

  public double getBackspinRPM(){
    return backspinEncoder.getVelocity();
  }

  public void setTurretDegrees(double degrees){
    double intendedTurretTicks = degrees*.11666666667;
    double currentTurretTicks = getTurretPosition();
    double ticksToMove = -(currentTurretTicks-intendedTurretTicks);
    double turretPower = Math.sqrt(ticksToMove)/Constants.turretAccelRate;

    setTurretPower(turretPower);

    SmartDashboard.putNumber("intended ticks", intendedTurretTicks);
    SmartDashboard.putNumber("current ticks", currentTurretTicks);
    SmartDashboard.putNumber("ticks to move", ticksToMove);
    SmartDashboard.putNumber("turret power", turretPower);
  }

  public SparkMaxPIDController getTurretPID(){
    return turretPID;
  }

}
