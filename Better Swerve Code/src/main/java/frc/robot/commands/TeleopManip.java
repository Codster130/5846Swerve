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
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;



public class TeleopManip extends CommandBase {
  private Intake m_Intake;
  private Swerve s_Swerve;
  private Joystick driver;
  private Shooter m_Shooter;
  private Vision m_Vision;
  private Uptake m_Uptake;
  private Joystick manip;
  private BallDetection m_BallDetection;
  private Turret m_Turret;

  private boolean uptakeRunning = false;
  private double desiredFlywheelRPM, desiredBackspinRPM, actualFlywheelRPM, actualBackspinRPM, flywheelPower, backspinPower;
  private boolean spinUpShooter = false;

  private boolean intakeIsRunning;
  private double intakeAngularVelocity;
  private double drivetrainForwardVelocity;
  private double relativeVelocity;
  private double intakePower;

  private double intendedTurretAngle;
  private double intendedTurretTicks;
  private double wrongBallAngleStorage;
  private boolean lastBallColorWrong;

  private boolean automatedTrackingOn;
  double x;
  double y;
  double area;

  /** Creates a new TeleopTurret. */
  public TeleopManip(Swerve s_Swerve, Intake m_Intake, Turret m_Turret, Uptake m_Uptake, Vision m_Vision, Shooter m_Shooter, BallDetection m_BallDetection, Joystick manip, Joystick driver) {
    this.s_Swerve = s_Swerve;
    this.m_Intake = m_Intake;
    this.driver = driver;
    this.m_Uptake = m_Uptake;
    this.m_Turret = m_Turret;
    this.m_Shooter = m_Shooter;
    this.m_Vision = m_Vision;
    this.m_BallDetection = m_BallDetection;
    this.manip = manip;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Turret, m_Uptake, m_Shooter, m_Vision, m_Intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    uptakeRunning = false;

    desiredFlywheelRPM = 0;
    desiredBackspinRPM = 0;
    actualFlywheelRPM = 0;
    actualBackspinRPM = 0;
    flywheelPower = 0;
    backspinPower = 0;
    spinUpShooter = false;

    intakeIsRunning = false;
    intakeAngularVelocity = 0;
    drivetrainForwardVelocity = 0;
    relativeVelocity = 0;
    intakePower = 0;

    intendedTurretAngle = 0;
    intendedTurretTicks = 0;
    wrongBallAngleStorage = 0;
    lastBallColorWrong = false;
    m_Turret.enable();

    automatedTrackingOn = false;
    x = 0;
    y = 0;
    area = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /***************\
    |Vision Controls| 
    \***************/
    
    if(manip.getRawButtonPressed(XboxController.Button.kStart.value)){
      automatedTrackingOn = !automatedTrackingOn;
    }
    if(automatedTrackingOn){
      x = m_Vision.getLimelightX();
      y = m_Vision.getLimelightY();
      area = m_Vision.getLimelightArea();
    }

    /***************\
    |Intake Controls| 
    \***************/
    
    intakeAngularVelocity = Math.abs((2*Math.PI*m_Intake.getIntakeVelocity()/12)/60/6);
    drivetrainForwardVelocity = s_Swerve.getForwardVelocity();
    relativeVelocity = intakeAngularVelocity-drivetrainForwardVelocity;

    if(driver.getRawButtonPressed(XboxController.Button.kA.value)){
      intakeIsRunning = !intakeIsRunning;
    }
    
    if(driver.getRawButton(XboxController.Button.kY.value)){
      intakePower = -1;
    }else if(intakeIsRunning){
      intakePower = 1;
    }else{
      intakePower = 0;
    }
    m_Intake.setIntakePower(-intakePower);

    /****************\
    |Shooter Controls| 
    \****************/ 

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

    /***************\
    |Uptake Controls| 
    \***************/ 

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
    
    /***************\
    |Turret Controls| 
    \***************/ 
    
    if(!automatedTrackingOn){
      if(manip.getRawButton(XboxController.Button.kLeftBumper.value)){
        intendedTurretAngle-=1;
      }else if(manip.getRawButton(XboxController.Button.kRightBumper.value)){
        intendedTurretAngle+=1;
      }else if(manip.getRawAxis(XboxController.Axis.kLeftTrigger.value)>.01){
        intendedTurretAngle-=manip.getRawAxis(XboxController.Axis.kLeftTrigger.value)*5;
      }else if(manip.getRawAxis(XboxController.Axis.kRightTrigger.value)>.01){
        intendedTurretAngle+=manip.getRawAxis(XboxController.Axis.kRightTrigger.value)*5;
      }
    } 
    
    else{
      if(x!=0){
        intendedTurretAngle+=x;
      }
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


    SmartDashboard.putNumber("Intake velocity", intakeAngularVelocity);
    SmartDashboard.putNumber("Robot velocity", drivetrainForwardVelocity);
    SmartDashboard.putNumber("Intake power", intakePower);
    SmartDashboard.putNumber("Relative velocity", relativeVelocity);
    SmartDashboard.putNumber("Actual flywheel RPM", actualFlywheelRPM);
    SmartDashboard.putNumber("Actual backspin RPM", actualBackspinRPM);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Turret angle", intendedTurretAngle);
    SmartDashboard.putNumber("Turret ticks", intendedTurretTicks);
    SmartDashboard.putNumber("Turret setpoint", m_Turret.getSetpoint());
    SmartDashboard.putNumber("Turret reading", m_Turret.getMeasurement());
    SmartDashboard.putBoolean("Turret on", m_Turret.isEnabled());

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
