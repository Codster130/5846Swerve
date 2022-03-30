// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.SetShooterRPM;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Uptake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GeneralAuto extends SequentialCommandGroup {
  Swerve s_Swerve;
  Uptake m_Uptake;
  Turret m_Turret;
  Shooter m_Shooter;
  
  /** Creates a new BlueRightAuto. */
  public GeneralAuto(Swerve s_Swerve, Shooter m_Shooter, Turret m_Turret, Uptake m_Uptake) {
    this.s_Swerve = s_Swerve;
    this.m_Shooter = m_Shooter;
    this.m_Turret = m_Turret;
    this.m_Uptake = m_Uptake;


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
     addCommands(
                //new InstantCommand(() -> m_Turret.setSetpoint(178)), 
                //new SetShooterRPM(750, 1500, m_Shooter),
                new InstantCommand(() -> m_Shooter.setFlywheelPower(.145)), 
                new InstantCommand(() -> m_Shooter.setBackspinPower(.3)),
                new WaitCommand(3), 
                new InstantCommand(() -> m_Uptake.setFeedPower(1)),
                new InstantCommand(() -> m_Uptake.setBeltPower(1)),
                new WaitCommand(1),
                new InstantCommand(() -> m_Uptake.setFeedPower(0)),
                new InstantCommand(() -> m_Shooter.setFlywheelPower(0)),
                new InstantCommand(() -> m_Uptake.setBeltPower(0)),
                new InstantCommand(() -> m_Shooter.setBackspinPower(0)), 
                new InstantCommand(() -> s_Swerve.drive(-.25, 0, 0, false)) 
                // new WaitCommand(3),
                // new InstantCommand(() -> s_Swerve.drive(0, 0, 0, false))
    );
  }
}
