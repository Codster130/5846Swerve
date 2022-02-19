// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
  NetworkTableEntry ta;
  NetworkTableEntry tx;
  NetworkTableEntry ty;

  /** Creates a new Vision. */
  public Vision() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public double getLimelightX(){
    return tx.getDouble(0.0);
  }

  public double getLimelightY(){
    return ty.getDouble(0.0);
  }

  public double getLimelightArea(){
    return ta.getDouble(0.0);
  }
}
