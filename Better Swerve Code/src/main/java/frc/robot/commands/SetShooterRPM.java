// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetShooterRPM extends CommandBase {
  private Shooter m_Shooter;

  private boolean finished = false;
  private double desiredFlywheelRPM, desiredBackspinRPM, actualFlywheelRPM, actualBackspinRPM, flywheelPower, backspinPower;

  /** Creates a new revUpShooter. */
  public SetShooterRPM(double flywheelRPM, double backspinRPM, Shooter m_Shooter) {
    this.m_Shooter = m_Shooter;
    this.desiredFlywheelRPM = flywheelRPM;
    this.desiredBackspinRPM = backspinRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_Shooter.setFlywheelPower(.5);
   m_Shooter.setBackspinPower(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(actualFlywheelRPM < desiredFlywheelRPM&&flywheelPower<1){
      flywheelPower+=MathUtil.clamp(Math.abs(actualFlywheelRPM-desiredFlywheelRPM)/100000, .0001, .01);
    }else if(actualFlywheelRPM > desiredFlywheelRPM&& flywheelPower>0){
      flywheelPower-=MathUtil.clamp(Math.abs(actualFlywheelRPM-desiredFlywheelRPM)/100000, .0001, .01);
    }
    
    if(actualBackspinRPM < desiredBackspinRPM&&backspinPower<1){
      backspinPower+=MathUtil.clamp(Math.abs(actualBackspinRPM-desiredBackspinRPM)/100000, .0001, .01);
    }else if(actualBackspinRPM > desiredBackspinRPM&&backspinPower>0){
      backspinPower-=MathUtil.clamp(Math.abs(actualBackspinRPM-desiredBackspinRPM)/100000, .0001, .01);
    }

    m_Shooter.setFlywheelPower(.5);
    m_Shooter.setBackspinPower(.5);

    if(Math.abs(desiredFlywheelRPM-actualFlywheelRPM)<8 && Math.abs(desiredBackspinRPM-actualBackspinRPM)<8){
      finished = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
