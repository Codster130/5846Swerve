// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import com.revrobotics.CANSparkMax;


public class TeleopShooter extends CommandBase {
  /** Creates a new TeleopShooter. */
  private Shooter m_Shooter;
  private Vision m_Vision;
  private Joystick driver;
  private Joystick manip;
  private PIDController turretAnglePID;

  private double intendedTurretAngle;
  private double intendedTurretTicks;
  private double turretPower;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  //0.20408163265 degrees per tick or 4.9 ticks per degree

  public TeleopShooter(Shooter m_Shooter, Vision m_Vision, Joystick driver, Joystick manip) {
    this.m_Shooter = m_Shooter;
    this.m_Vision = m_Vision;
    this.driver = driver;
    this.manip = manip;
    turretAnglePID = new PIDController(.05, 0, 0);
    
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

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

    if(manip.getRawButton(XboxController.Button.kLeftBumper.value)){
      intendedTurretAngle+=.5;
    }else if(manip.getRawButton(XboxController.Button.kRightBumper.value)){
      intendedTurretAngle-=.5;
    }

    if(intendedTurretAngle>=180){
      intendedTurretAngle-=360;
    }else if(intendedTurretAngle<-180){
      intendedTurretAngle+=360;
    }

    m_Shooter.setTurretDegrees(intendedTurretAngle);

    m_Shooter.setBackspinPower(.25);
    m_Shooter.setFlywheelPower(.25);
    m_Shooter.getBackspinRPM();
    m_Shooter.getFlywheelRPM();

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putNumber("Turret Angle", intendedTurretAngle);
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
