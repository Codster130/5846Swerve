// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallDetection extends SubsystemBase {
  ColorSensorV3 topColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  Rev2mDistanceSensor bottomBallSensor = new Rev2mDistanceSensor(Port.kMXP);
  
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = new Color(0.185, 0.407, 0.407);
  private final Color kRedTarget = new Color(0.466, 0.367, 0.166);
  /** Creates a new BallDetection. */
  public BallDetection() {  
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    bottomBallSensor.setAutomaticMode(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public String getTopColor(){
    Color detectedColor = topColorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget && match.confidence > .95) {
      colorString = "Blue";
    } else if (match.color == kRedTarget && match.confidence > .95) {
      colorString = "Red";
    } else {
      colorString = "Unknown";
    }
    return colorString;
  }

  public boolean ballMatchesAlliance(String ballColor){
    return DriverStation.getAlliance().toString().equals(ballColor);
  }

  public boolean wrongTopBall(){
    return (!ballMatchesAlliance(getTopColor())) && !getTopColor().equals("Unknown");
  }

  public boolean seesTopBall(){
    return getTopColor().equals("Red")||getTopColor().equals("Blue");
  }

  public boolean seesBottomBall(){
    return bottomBallSensor.getRange(Unit.kInches)<2;
  }

  public double distSensorDistance(){
    return bottomBallSensor.getRange(Unit.kInches);
  }

}
