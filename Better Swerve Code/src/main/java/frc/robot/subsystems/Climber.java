// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax leftClimber;
  CANSparkMax rightClimber;
  /** Creates a new Climber. */
  public Climber() {
    leftClimber = new CANSparkMax(Constants.leftClimberMotorId, MotorType.kBrushless);
    rightClimber = new CANSparkMax(Constants.rightClimberMotorId, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setClimberPower(double power){
    leftClimber.set(-power);
    rightClimber.set(power);
  }
}
