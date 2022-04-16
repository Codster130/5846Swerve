// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.SetShooterRPM;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Uptake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallAuto extends SequentialCommandGroup {
  Swerve s_Swerve;
  Uptake m_Uptake;
  Turret m_Turret;
  Shooter m_Shooter;
  Intake m_Intake;
  
  /** Creates a new BlueRightAuto. */
  public OneBallAuto(Swerve s_Swerve, Shooter m_Shooter, Turret m_Turret, Uptake m_Uptake, Intake m_Intake) {
    this.s_Swerve = s_Swerve;
    this.m_Shooter = m_Shooter;
    this.m_Turret = m_Turret;
    this.m_Uptake = m_Uptake;
    this.m_Intake = m_Intake;

    m_Turret.setPosition(179*.1945);


    // An ExampleCommand will run in autonomous
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(Constants.Swerve.swerveKinematics);

              // 2. Generate trajectory
        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
                  new Translation2d(.5, .01)),
          new Pose2d(2.5, 0, Rotation2d.fromDegrees(0)),
          trajectoryConfig);

        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(1, 0, new Rotation2d(0)),
          List.of(
                  new Translation2d(.5, .01)),
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
          trajectoryConfig);

        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
                  new Translation2d(.5, .01)),
          new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)),
          trajectoryConfig);

          // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

         // 4. Construct command to follow trajectory
         SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
          trajectory1,
          s_Swerve::getPose,
          Constants.Swerve.swerveKinematics,
          xController,
          yController,
          thetaController,
          s_Swerve::setModuleStates,
          s_Swerve);

          SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
          trajectory2,
          s_Swerve::getPose,
          Constants.Swerve.swerveKinematics,
          xController,
          yController,
          thetaController,
          s_Swerve::setModuleStates,
          s_Swerve);

          SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
          trajectory3,
          s_Swerve::getPose,
          Constants.Swerve.swerveKinematics,
          xController,
          yController,
          thetaController,
          s_Swerve::setModuleStates,
          s_Swerve);

          


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
     addCommands(
                //Enable and position turret
                new InstantCommand(()-> m_Turret.disable()),
                new InstantCommand(() -> m_Turret.setSetpoint(179*.1945)), 
                
                //Rev up shooter
                new InstantCommand(() -> m_Shooter.setFlywheelPower(.145)), //.145, .38
                new InstantCommand(() -> m_Shooter.setBackspinPower(.3)), //.3, .35
                new WaitCommand(3), 

                //Shoot
                new InstantCommand(() -> m_Uptake.setBeltPower(1)),
                new InstantCommand(() -> m_Uptake.setFeedPower(1)),

                //Stop everything
                new WaitCommand(1),
                new InstantCommand(() -> m_Uptake.setFeedPower(0)),
                new InstantCommand(() -> m_Shooter.setFlywheelPower(0)),
                new InstantCommand(() -> m_Uptake.setBeltPower(0)),
                new InstantCommand(() -> m_Shooter.setBackspinPower(0)),

                //Drive out
                new SequentialCommandGroup(
                  new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialPose())), swerveControllerCommand1,
                  new InstantCommand(() -> s_Swerve.drive(0, 0, 0, false))
                 )
                
    );
  }
}
