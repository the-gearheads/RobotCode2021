/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final CANSparkMax elevatorUpper;
  private final CANSparkMax elevatorLower;

  public Elevator() {
    elevatorUpper = new CANSparkMax(15, MotorType.kBrushless);
    elevatorLower = new CANSparkMax(11, MotorType.kBrushless);

    elevatorLower.setInverted(true);

    elevatorUpper.setIdleMode(IdleMode.kBrake);
    elevatorLower.setIdleMode(IdleMode.kBrake);
  }

  public void elevate(double upper, double lower) {
    elevatorUpper.set(upper);
    elevatorLower.set(lower);
  }

  public void elevate() {
    elevate(.4, .4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
