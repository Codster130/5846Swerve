// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class TeleopIntake extends CommandBase {

  private Intake m_Intake;
  private Swerve s_Swerve;
  private Joystick driver;
  private Joystick manip;
  private boolean intakeIsRunning;
  private double intakeAngularVelocity;
  private double drivetrainForwardVelocity;
  private double relativeVelocity;
  private double power;
  /** Creates a new TeleopIntake. */
  public TeleopIntake(Swerve s_Swerve, Intake m_Intake, Joystick driver, Joystick manip) {
    this.s_Swerve = s_Swerve;
    this.m_Intake = m_Intake;
    this.driver = driver;
    this.manip = manip;
    intakeIsRunning = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Intake Velocity", intakeAngularVelocity);
    SmartDashboard.putNumber("Robot Velocity", drivetrainForwardVelocity);
    SmartDashboard.putNumber("Intake power", power);
    SmartDashboard.putNumber("Relative velocity", relativeVelocity);
    

    intakeAngularVelocity = Math.abs((2*Math.PI*m_Intake.getIntakeVelocity()/12)/60/6);
    drivetrainForwardVelocity = s_Swerve.getForwardVelocity();
    relativeVelocity = intakeAngularVelocity-drivetrainForwardVelocity;

    if(driver.getRawButtonPressed(XboxController.Button.kA.value)){
      intakeIsRunning = !intakeIsRunning;
    }
    
    if(driver.getRawButton(XboxController.Button.kY.value)){
      // if(relativeVelocity < Constants.intakeRelativeSpeed-.01&&power<1){
      //   power+=.01;
      // }else if(relativeVelocity > Constants.intakeRelativeSpeed+.01&&power>0){
      //   power-=.01;
      // }
      power = -1;
      
    }else if(intakeIsRunning){
      power = 1;
    }else{
      power = 0;
    }
    m_Intake.setIntakePower(-power);
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
