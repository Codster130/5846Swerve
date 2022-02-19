// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  CANSparkMax flywheelMotor;
  CANSparkMax turretMotor;
  RelativeEncoder flywheelEncoder;

  /** Creates a new Shooter. */
  public Shooter() {
    flywheelMotor = new CANSparkMax(Constants.flywheelMotorID, MotorType.kBrushless);
    turretMotor = new CANSparkMax(Constants.turretMotorID, MotorType.kBrushless);
    flywheelMotor.enableVoltageCompensation(12);
    turretMotor.enableVoltageCompensation(12);
    flywheelEncoder = flywheelMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFlywheelPower(double power){
    flywheelMotor.set(power);
  }

  public double getFlywheelRPM(){
    return flywheelEncoder.getVelocity();
  }
}
