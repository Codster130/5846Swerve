// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick manipulator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton dY = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton dB = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton dX = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton dA = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton dLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton dRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton dRS = new JoystickButton(driver, XboxController.Button.kRightStick.value);
  private final JoystickButton dLS = new JoystickButton(driver, XboxController.Button.kLeftStick.value);

  /* Manipulator Buttons */
  private final JoystickButton mY = new JoystickButton(manipulator, XboxController.Button.kY.value);
  private final JoystickButton mB = new JoystickButton(manipulator, XboxController.Button.kB.value);
  private final JoystickButton mX = new JoystickButton(manipulator, XboxController.Button.kX.value);
  private final JoystickButton mA = new JoystickButton(manipulator, XboxController.Button.kA.value);
  private final JoystickButton mLB = new JoystickButton(manipulator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton mRB = new JoystickButton(manipulator, XboxController.Button.kRightBumper.value);
  private final JoystickButton mRS = new JoystickButton(manipulator, XboxController.Button.kRightStick.value);
  private final JoystickButton mLS = new JoystickButton(manipulator, XboxController.Button.kLeftStick.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  public static Intake m_intake = new Intake();
  Trajectory trajectory = new Trajectory();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  } 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    dY.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    dB.whenPressed(new DeployIntake());
    dX.whenPressed(new RetractIntake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = s_Swerve.loadTrajectoryFromFile("src/main/deploy/output/yes.wpilib.json");

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            s_Swerve::getPose, // Functional interface to feed supplier
            Constants.Swerve.swerveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    // Reset odometry to the starting pose of the trajectory.
    s_Swerve.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> s_Swerve.drive(0, 0, 0, false));
  }
}
