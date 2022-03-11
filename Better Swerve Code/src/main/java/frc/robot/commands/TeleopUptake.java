// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Uptake;

public class TeleopUptake extends CommandBase {
  private Uptake m_Uptake;
  private Joystick manip;
  private BallDetection m_BallDetection;

  boolean uptakeRunning = false;

  /** Creates a new UptakeCommand. */
  public TeleopUptake(Uptake m_Uptake, BallDetection m_BallDetection, Joystick manip){
    this.m_Uptake = m_Uptake;
    this.m_BallDetection = m_BallDetection;
    this.manip = manip;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(manip.getRawButtonPressed(XboxController.Button.kX.value)){
      uptakeRunning = !uptakeRunning;
    }

    SmartDashboard.putString("Color", m_BallDetection.getTopColor());

    if(uptakeRunning){
      m_Uptake.setBeltPower(1);
    }else if (manip.getRawButtonPressed(XboxController.Button.kB.value)){
      m_Uptake.setBeltPower(-.5);
    }else{
      m_Uptake.setBeltPower(0);
    }

    if(manip.getRawButton(XboxController.Button.kA.value)){
      m_Uptake.setFeedPower(1);
    }else{
      m_Uptake.setFeedPower(0);
    }
    SmartDashboard.updateValues();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
