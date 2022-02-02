// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Uptake extends SubsystemBase {
  
  //Left and right agitators (bottom wheels entering into uptake)
  VictorSPX leftAgitatorMotor;
  VictorSPX rightAgitatorMotor;

  //Front and back belt drivers
  VictorSPX frontBeltMotor;
  VictorSPX backBeltMotor;

  //Left and right flywheel feed motors (top of belt uptake)
  VictorSPX leftFeedMotor;
  VictorSPX rightFeedMotor;

  /** Creates a new Uptake. */
  public Uptake() {
    configureUptakeMotors();
    setVoltageCompensation(12, true);

    leftAgitatorMotor.follow(rightAgitatorMotor);
    backBeltMotor.follow(frontBeltMotor);
    leftFeedMotor.follow(rightFeedMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configureUptakeMotors(){
    //Initialize uptake motors
    leftAgitatorMotor = new VictorSPX(Constants.leftAgitatorMotorID);
    rightAgitatorMotor = new VictorSPX(Constants.rightAgitatorMotorID);
    frontBeltMotor = new VictorSPX(Constants.frontBeltMotorID);
    backBeltMotor = new VictorSPX(Constants.backBeltMotorID);
    leftFeedMotor = new VictorSPX(Constants.leftFeedMotorID);
    rightFeedMotor = new VictorSPX(Constants.rightFeedMotorID);
  }

  public void setVoltageCompensation(int voltage, boolean enable){
    leftAgitatorMotor.configVoltageCompSaturation(voltage);
    leftAgitatorMotor.enableVoltageCompensation(enable);

    rightAgitatorMotor.configVoltageCompSaturation(voltage);
    rightAgitatorMotor.enableVoltageCompensation(enable);

    frontBeltMotor.configVoltageCompSaturation(voltage);
    frontBeltMotor.enableVoltageCompensation(enable);

    backBeltMotor.configVoltageCompSaturation(voltage);
    backBeltMotor.enableVoltageCompensation(enable);

    leftFeedMotor.configVoltageCompSaturation(voltage);
    leftFeedMotor.enableVoltageCompensation(enable);

    rightFeedMotor.configVoltageCompSaturation(voltage);
    rightFeedMotor.enableVoltageCompensation(enable);
  }

  public void setAgitatorPower(double power){
    rightAgitatorMotor.set(ControlMode.PercentOutput, power);
  }

  public void setBeltPower(double power){
    frontBeltMotor.set(ControlMode.PercentOutput, power);
  }

  public void feedBall(double rotations){
    double targetPos = leftFeedMotor.getSelectedSensorPosition()+rotations*4096;
    leftFeedMotor.set(ControlMode.MotionMagic, targetPos);
  }

}
