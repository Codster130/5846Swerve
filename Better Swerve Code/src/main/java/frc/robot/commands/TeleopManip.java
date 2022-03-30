// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallDetection;
import frc.robot.subsystems.Climber;
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
  private Climber m_Climber;

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
  private boolean hasTwoBalls;

  private boolean automatedShootingOn;
  private boolean automatedTrackingOn;
  double x;
  double y;
  double area;

  /** Creates a new TeleopTurret. */
  public TeleopManip(Swerve s_Swerve, Intake m_Intake, Turret m_Turret, Uptake m_Uptake, Vision m_Vision, Shooter m_Shooter, Climber m_Climber, BallDetection m_BallDetection, Joystick manip, Joystick driver) {
    this.s_Swerve = s_Swerve;
    this.m_Intake = m_Intake;
    this.driver = driver;
    this.m_Uptake = m_Uptake;
    this.m_Turret = m_Turret;
    this.m_Shooter = m_Shooter;
    this.m_Vision = m_Vision;
    this.m_Climber = m_Climber;
    this.m_BallDetection = m_BallDetection;
    this.manip = manip;



    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Turret, m_Uptake, m_Shooter, m_Vision, m_Intake, m_Climber, m_BallDetection);

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
    hasTwoBalls = false;
    m_Turret.enable();

    automatedShootingOn = false;
    automatedTrackingOn = false;
    x = 0;
    y = 0;
    area = 0;

    m_Turret.setPosition(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  if(!DriverStation.isAutonomousEnabled()){  
    /*************************\
    |Shooting mode start check| 
    \*************************/
    
    if(manip.getRawButtonPressed(XboxController.Button.kStart.value)){
      automatedShootingOn = !automatedShootingOn;
      automatedTrackingOn = automatedShootingOn;
    }

    hasTwoBalls = !m_BallDetection.getTopColor().equals("Unknown")&& m_BallDetection.seesBottomBall();

    /****************\
    |Climber Controls| 
    \****************/
    if(manip.getPOV() == 0){
      m_Climber.setClimberPower(1);
    }else if(manip.getPOV() == 180){
      m_Climber.setClimberPower(-1);
    }else{
      m_Climber.setClimberPower(0);
    }

    /***************\
    |Vision Controls| 
    \***************/
    
    x = m_Vision.getLimelightX();
    y = m_Vision.getLimelightY();
    area = m_Vision.getLimelightArea();

    if(automatedTrackingOn){
      m_Vision.setLimelightLED(3);
    }else{
      m_Vision.setLimelightLED(1);
    }

    /***************\
    |Intake Controls| 
    \***************/
    
    intakeAngularVelocity = Math.abs((2*Math.PI*m_Intake.getIntakeVelocity()/12)/60/6);
    drivetrainForwardVelocity = s_Swerve.getForwardVelocity();
    relativeVelocity = intakeAngularVelocity-drivetrainForwardVelocity;

    if(driver.getRawButtonPressed(XboxController.Button.kX.value)||driver.getRawButtonPressed(XboxController.Button.kA.value)){
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
    
    //Tarmac edge: 2400, 3360; Low goal: 750, 2500; Bottom wall: 2080, 2880
    if(manip.getRawButtonPressed(XboxController.Button.kLeftStick.value)){
      //high goal RPMs
      desiredFlywheelRPM = 2080;
      desiredBackspinRPM = 2880;
      spinUpShooter = true;
    }

    if(manip.getRawButtonPressed(XboxController.Button.kRightStick.value)){
      //low goal RPMs
      desiredFlywheelRPM = 750;
      desiredBackspinRPM = 1500;
      spinUpShooter = true;
    }

    actualFlywheelRPM = m_Shooter.getFlywheelRPM();
    actualBackspinRPM = m_Shooter.getBackspinRPM();

    if(manip.getRawButtonPressed(XboxController.Button.kY.value)){
      spinUpShooter = !spinUpShooter;
    }else if(hasTwoBalls&&automatedShootingOn){
      spinUpShooter = true;
    }

    if(m_BallDetection.wrongTopBall()&&automatedShootingOn){
      desiredFlywheelRPM = 500;
      desiredBackspinRPM = 1000;
    }


    if(spinUpShooter){
      if(actualFlywheelRPM < desiredFlywheelRPM&&flywheelPower<1){
        flywheelPower+=MathUtil.clamp(Math.abs(actualFlywheelRPM-desiredFlywheelRPM)/100000, .0001, .01);
      }else if(actualFlywheelRPM > desiredFlywheelRPM&& flywheelPower>0){
        flywheelPower-=MathUtil.clamp(Math.abs(actualFlywheelRPM-desiredFlywheelRPM)/100000, .0001, .01);
      }
    }else{
        flywheelPower = 0;
    }

    if(spinUpShooter){
      if(actualBackspinRPM < desiredBackspinRPM&&backspinPower<1){
        backspinPower+=MathUtil.clamp(Math.abs(actualBackspinRPM-desiredBackspinRPM)/100000, .0001, .01);
      }else if(actualBackspinRPM > desiredBackspinRPM&&backspinPower>0){
        backspinPower-=MathUtil.clamp(Math.abs(actualBackspinRPM-desiredBackspinRPM)/100000, .0001, .01);
      }
    }else{
        backspinPower = 0;
    }

    m_Shooter.setFlywheelPower(flywheelPower);
    m_Shooter.setBackspinPower(backspinPower);

    /***************\
    |Uptake Controls| 
    \***************/ 

    if(manip.getRawButtonPressed(XboxController.Button.kX.value)){
      uptakeRunning = !uptakeRunning;
    }

    if(hasTwoBalls&&automatedShootingOn){
      uptakeRunning = false;
    }

    SmartDashboard.putString("Color", m_BallDetection.getTopColor());

    if(uptakeRunning){
      m_Uptake.setBeltPower(1);
    }else if (manip.getRawButton(XboxController.Button.kB.value)){
      m_Uptake.setBeltPower(-.5);
    }else{
      m_Uptake.setBeltPower(0);
    }

    if(manip.getRawButton(XboxController.Button.kA.value) && Math.abs(desiredFlywheelRPM-actualFlywheelRPM)<20 && Math.abs(desiredBackspinRPM-actualBackspinRPM)<20 && actualBackspinRPM+actualFlywheelRPM>20){
      m_Uptake.setFeedPower(1);
    }else{
      m_Uptake.setFeedPower(0);
    }
    
    /***************\
    |Turret Controls| 
    \***************/ 
    

    if(!automatedShootingOn){
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
    
    else if (automatedTrackingOn&&automatedShootingOn){
      if(x!=0){
        intendedTurretAngle+=x/5;
      }else if(x==0){
        intendedTurretAngle+=3;
      }
    }

    if(automatedShootingOn){
      if(m_BallDetection.wrongTopBall()&&m_BallDetection.wrongTopBall()!=lastBallColorWrong){
        lastBallColorWrong = true;
        automatedTrackingOn = false;
        intendedTurretAngle+=90;
        if(intendedTurretAngle>=180){
          intendedTurretAngle-=180;
        }
      }else if((!m_BallDetection.wrongTopBall())&&m_BallDetection.wrongTopBall()!=lastBallColorWrong){
        lastBallColorWrong = false;
        automatedTrackingOn = true;
      }
    }
    
    if(intendedTurretAngle>=180){
      intendedTurretAngle-=360;
    }else if(intendedTurretAngle<-180){
      intendedTurretAngle+=360;
    }

    intendedTurretTicks = intendedTurretAngle*.1945;

    m_Turret.setSetpoint(intendedTurretTicks);

    SmartDashboard.putNumber("Flywheel target", desiredFlywheelRPM);
    SmartDashboard.putNumber("Backspin power", backspinPower);
    SmartDashboard.putNumber("Flywheel power", flywheelPower);
    SmartDashboard.putBoolean("Distance sensor sees", m_BallDetection.seesBottomBall());
    SmartDashboard.putNumber("Distance sensor distance", m_BallDetection.distSensorDistance());
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
