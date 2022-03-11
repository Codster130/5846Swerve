// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TeleopTurret extends CommandBase {
  private Turret m_Turret;
  private BallDetection m_BallDetection;
  private Joystick manip;
  
  private double intendedTurretAngle;
  private double intendedTurretTicks;
  private double turretPower;

  private double wrongBallAngleStorage;
  private boolean lastBallColorWrong;

  private int loopCounter;

  /** Creates a new TeleopTurret. */
  public TeleopTurret(Turret m_Turret, Vision m_Vision, BallDetection m_BallDetection, Joystick manip) {
    this.m_Turret = m_Turret;
    this.m_BallDetection = m_BallDetection;
    this.manip = manip;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intendedTurretAngle = 0;
    wrongBallAngleStorage = 0;
    loopCounter = 0;
    lastBallColorWrong = false;
    m_Turret.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCounter+=1;
    if(manip.getRawButton(XboxController.Button.kLeftBumper.value)){
      intendedTurretAngle-=1;
    }else if(manip.getRawButton(XboxController.Button.kRightBumper.value)){
      intendedTurretAngle+=1;
    }else if(manip.getRawAxis(XboxController.Axis.kLeftTrigger.value)>.01){
      intendedTurretAngle-=manip.getRawAxis(XboxController.Axis.kLeftTrigger.value)*5;
    }else if(manip.getRawAxis(XboxController.Axis.kRightTrigger.value)>.01){
      intendedTurretAngle+=manip.getRawAxis(XboxController.Axis.kRightTrigger.value)*5;
    }

    if(m_BallDetection.wrongTopBall()&&m_BallDetection.wrongTopBall()!=lastBallColorWrong){
      lastBallColorWrong = true;
      wrongBallAngleStorage = intendedTurretAngle;
      intendedTurretAngle+=90;
      if(intendedTurretAngle>=180){
        intendedTurretAngle-=180;
      }
    }else if((!m_BallDetection.wrongTopBall())&&m_BallDetection.wrongTopBall()!=lastBallColorWrong){
      lastBallColorWrong = false;
      intendedTurretAngle = wrongBallAngleStorage;
    }
    
    

    if(intendedTurretAngle>=180){
      intendedTurretAngle-=360;
    }else if(intendedTurretAngle<-180){
      intendedTurretAngle+=360;
    }
    intendedTurretTicks = intendedTurretAngle*.1945;

    m_Turret.setSetpoint(intendedTurretTicks);

    SmartDashboard.putNumber("Turret Angle", intendedTurretAngle);
    SmartDashboard.putNumber("Turret Ticks", intendedTurretTicks);
    SmartDashboard.putNumber("Turret setpoint", m_Turret.getSetpoint());
    SmartDashboard.putNumber("Turret reading", m_Turret.getMeasurement());
    SmartDashboard.putBoolean("PID On", m_Turret.isEnabled());

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
