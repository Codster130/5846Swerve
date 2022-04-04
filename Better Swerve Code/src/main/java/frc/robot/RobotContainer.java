// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final Joystick manip = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  public final JoystickButton dY = new JoystickButton(driver, XboxController.Button.kY.value); //Reverse intake
  private final JoystickButton dB = new JoystickButton(driver, XboxController.Button.kB.value); //Zero gyro
  private final JoystickButton dX = new JoystickButton(driver, XboxController.Button.kX.value); //Toggle intake
  private final JoystickButton dA = new JoystickButton(driver, XboxController.Button.kA.value); //Run intake
  private final JoystickButton dLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton dRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton dRS = new JoystickButton(driver, XboxController.Button.kRightStick.value);
  private final JoystickButton dLS = new JoystickButton(driver, XboxController.Button.kLeftStick.value);

  /* Manipulator Buttons */
  private final JoystickButton mY = new JoystickButton(manip, XboxController.Button.kY.value); //Spin up flywheel for top
  private final JoystickButton mB = new JoystickButton(manip, XboxController.Button.kB.value); //Spin up flywheel for bottom
  private final JoystickButton mX = new JoystickButton(manip, XboxController.Button.kX.value); //Uptake up
  private final JoystickButton mA = new JoystickButton(manip, XboxController.Button.kA.value); //Shoot
  private final JoystickButton mSt = new JoystickButton(manip, XboxController.Button.kStart.value); //Toggle vision
  private final JoystickButton mLB = new JoystickButton(manip, XboxController.Button.kLeftBumper.value); //Turret left
  private final JoystickButton mRB = new JoystickButton(manip, XboxController.Button.kRightBumper.value); //Turret right
  private final JoystickButton mRS = new JoystickButton(manip, XboxController.Button.kRightStick.value); 
  private final JoystickButton mLS = new JoystickButton(manip, XboxController.Button.kLeftStick.value);


  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final Turret m_Turret = new Turret();
  public final Intake m_Intake = new Intake();
  public final Uptake m_Uptake = new Uptake();
  public final Shooter m_Shooter = new Shooter();
  public final Vision m_Vision = new Vision();
  public final Climber m_Climber = new Climber();
  public final BallDetection m_BallDetection = new BallDetection();
  Trajectory trajectory = new Trajectory();
  UsbCamera usb1 = CameraServer.startAutomaticCapture(0);

  // A simple auto routine that drives forward a specified distance, and then stops.
  private final Command m_simpleAuto = new TwoBallAuto(s_Swerve, m_Shooter, m_Turret, m_Uptake, m_Intake);

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  private final Command m_complexAuto = new exampleAuto(s_Swerve);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, manip, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
    m_Intake.setDefaultCommand(new TeleopManip(s_Swerve, m_Intake, m_Turret, m_Uptake, m_Vision, m_Shooter, m_Climber, m_BallDetection, manip, driver));

    usb1.setResolution(160, 120);
    usb1.setBrightness(25);

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("Complex Auto", m_complexAuto);

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser); 


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
    dB.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    dX.whenPressed(new InstantCommand(() -> m_Intake.toggleIntake()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // 1. Create trajectory settings
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    //   AutoConstants.kMaxSpeedMetersPerSecond,
    //   AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //           .setKinematics(Constants.Swerve.swerveKinematics);

    //           // 2. Generate trajectory
    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //       new Pose2d(0, 0, new Rotation2d(0)),
    //       List.of(
    //               new Translation2d(1, .1)),
    //       new Pose2d(2, 0, Rotation2d.fromDegrees(180)),
    //       trajectoryConfig);

    //       // 3. Define PID controllers for tracking trajectory
    //     PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    //     PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    //     ProfiledPIDController thetaController = new ProfiledPIDController(
    //             AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //      // 4. Construct command to follow trajectory
    //      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //       trajectory,
    //       s_Swerve::getPose,
    //       Constants.Swerve.swerveKinematics,
    //       xController,
    //       yController,
    //       thetaController,
    //       s_Swerve::setModuleStates,
    //       s_Swerve);

    //       // 5. Add some init and wrap-up, and return everything
    //     return new SequentialCommandGroup(
    //       new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
    //       swerveControllerCommand,
    //       new InstantCommand(() -> s_Swerve.drive(0, 0, 0, false)));

    return m_chooser.getSelected();
    // return new GeneralAuto(s_Swerve, m_Shooter, m_Turret, m_Uptake);
  }
}
