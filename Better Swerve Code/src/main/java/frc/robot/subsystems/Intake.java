// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  DoubleSolenoid intakeSolenoid = null;
  TalonFX intakeMotor = null;

  /** Creates a new Intake. */
  public Intake() {
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,1);
    intakeMotor = new TalonFX(Constants.intakeMotorID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakePower(double power){
    intakeMotor.set(ControlMode.PercentOutput, power);
  }

  public void extendIntake(){
    intakeSolenoid.set(Value.kForward);
  } 

  public void retractIntake(){
    intakeSolenoid.set(Value.kReverse);
  }

}
