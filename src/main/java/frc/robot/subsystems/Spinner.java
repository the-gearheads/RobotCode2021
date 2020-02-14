/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
  private final CANSparkMax spinnerMotor;
  private ColorSensorV3 colorSensor;
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final HashMap<Color, String> map = new HashMap<>();

  public Spinner() {
    spinnerMotor = new CANSparkMax(15, MotorType.kBrushless);
    colorSensor = new ColorSensorV3(Port.kMXP);
    map.put(kBlueTarget, "Red");
    map.put(kYellowTarget, "Green");
    map.put(kRedTarget, "Blue");
    map.put(kGreenTarget, "Yellow");
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  public void spinClockwise() {
    spinnerMotor.set(-0.25);
  }

  public void spinCClockwise() {
    spinnerMotor.set(0.25);
  }

  public void stop() {
    spinnerMotor.set(0);
  }

  public void periodic() {
    SmartDashboard.putString("Die Farbe", getColor());
  }

  public String getColor() {
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    return map.get(match.color);
  }
}
