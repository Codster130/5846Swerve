// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class TeleopShooter extends CommandBase {
  /** Creates a new TeleopShooter. */
  private Shooter m_Shooter;
  private Vision m_Vision;
  private Joystick driver;
  private Joystick manip;

  public TeleopShooter(Shooter m_Shooter, Vision m_Vision, Joystick driver, Joystick manip) {
    this.m_Shooter = m_Shooter;
    this.m_Vision = m_Vision;
    this.driver = driver;
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
