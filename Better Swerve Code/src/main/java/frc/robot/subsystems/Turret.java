// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Turret extends PIDSubsystem {
  CANSparkMax turretMotor = new CANSparkMax(Constants.turretMotorID, MotorType.kBrushless);
  RelativeEncoder turretEncoder = turretMotor.getEncoder();
  /** Creates a new Turret. */
  public Turret() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    getController().setTolerance(1, 1);
    setSetpoint(0);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    turretMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return turretEncoder.getPosition();
  }
}
