// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;


public class TeleopShooter extends CommandBase {
  /** Creates a new TeleopShooter. */
  private Shooter m_Shooter;
  private Vision m_Vision;
  private Joystick manip;
  private BallDetection m_BallDetection;
  

  private double desiredFlywheelRPM, desiredBackspinRPM, actualFlywheelRPM, actualBackspinRPM, flywheelPower, backspinPower;
  private boolean spinUpShooter = false;

  public TeleopShooter(Shooter m_Shooter, Vision m_Vision, BallDetection M_BallDetection, Joystick driver, Joystick manip) {
    this.m_Shooter = m_Shooter;
    this.m_Vision = m_Vision;
    this.m_BallDetection = m_BallDetection;
    this.manip = manip;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
    addRequirements(m_Vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = m_Vision.getLimelightX();
    double y = m_Vision.getLimelightY();
    double area = m_Vision.getLimelightArea();

    desiredFlywheelRPM = SmartDashboard.getNumber("Desired Flywheel RPM", 100);
    desiredBackspinRPM = SmartDashboard.getNumber("Desired Backspin RPM", 100);
    
    actualFlywheelRPM = m_Shooter.getFlywheelRPM();
    actualBackspinRPM = m_Shooter.getBackspinRPM();

    if(manip.getRawButtonPressed(XboxController.Button.kX.value)){
      spinUpShooter = !spinUpShooter;
    }

    if(spinUpShooter){
      if(actualFlywheelRPM < desiredFlywheelRPM-.01&&flywheelPower<1){
        flywheelPower+=.01;
      }else if(actualFlywheelRPM > desiredFlywheelRPM+.01&& flywheelPower>0){
        flywheelPower-=.01;
      }
    }else{
      flywheelPower = 0;
    }

    if(spinUpShooter){
      if(actualBackspinRPM < desiredBackspinRPM-.01&&backspinPower<1){
        flywheelPower+=.01;
      }else if(actualBackspinRPM > desiredBackspinRPM+.01&& backspinPower>0){
        backspinPower-=.01;
      }
    }else{
      backspinPower = 0;
    }
    
    SmartDashboard.putNumber("Actual Flywheel RPM", actualFlywheelRPM);
    SmartDashboard.putNumber("Actual Backspin RPM", actualBackspinRPM);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

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
