// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  DoubleSolenoid leftIntakeSolenoid = null;
  DoubleSolenoid rightIntakeSolenoid = null;
  CANSparkMax intakeMotor = null;

  /** Creates a new Intake. */
  public Intake() {
    leftIntakeSolenoid = new DoubleSolenoid(13, PneumaticsModuleType.REVPH, 0, 1);
    rightIntakeSolenoid = new DoubleSolenoid(13, PneumaticsModuleType.REVPH, 2, 3);
    intakeMotor = new CANSparkMax(Constants.intakeMotorID, MotorType.kBrushless);
    leftIntakeSolenoid.set(Value.kReverse);
    rightIntakeSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakePower(double power){
    intakeMotor.set(power);
  }

  public void toggleIntake(){
    leftIntakeSolenoid.toggle();
    rightIntakeSolenoid.toggle();
  }

  public double getIntakeVelocity(){
    return intakeMotor.getEncoder().getVelocity();
  }

}
