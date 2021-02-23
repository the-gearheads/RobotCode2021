/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.group.BlockedElevate;

public class Elevator extends SubsystemBase {

  private final CANSparkMax elevatorUpper;
  private final CANSparkMax elevatorLower;
  private final CANEncoder upperEncoder;
  private final CANEncoder lowerEncoder;

  private double lowerPosition;
  private double upperPosition;

  public Elevator() {
    elevatorUpper = new CANSparkMax(15, MotorType.kBrushless);
    elevatorLower = new CANSparkMax(11, MotorType.kBrushless);
    upperEncoder = elevatorUpper.getEncoder();
    lowerEncoder = elevatorLower.getEncoder();

    elevatorLower.setInverted(true);
    // setDefaultCommand(new BlockedElevate(this, RobotContainer.getIntake()));
  }

  public void zero() {
    upperEncoder.setPosition(0);
    lowerEncoder.setPosition(0);
  }

  public void elevate(double upper, double lower) {
    upper(upper);
    lower(lower);
  }

  public void lower(double speed) {
    elevatorLower.set(speed);
  }

  public void upper(double speed) {
    elevatorUpper.set(speed);
  }

  public void elevate() {
    // elevate(.6, .35);
    elevate(.3, .25);
  }

  public double getUpperPosition() {
    return upperPosition;
  }

  public double getLowerPosition() {
    return lowerPosition;
  }

  public void setBrake() {
    elevatorUpper.setIdleMode(IdleMode.kBrake);
    elevatorLower.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    elevatorUpper.setIdleMode(IdleMode.kCoast);
    elevatorLower.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    upperPosition = upperEncoder.getPosition();
    lowerPosition = lowerEncoder.getPosition();
  }
}
