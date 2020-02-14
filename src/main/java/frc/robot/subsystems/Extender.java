/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {
  /**
   * Creates a new Extender.
   */
  private final CANSparkMax leftSpin;
  private final CANSparkMax rightSpin;

  public Extender() {
    leftSpin = new CANSparkMax(22, MotorType.kBrushless);
    rightSpin = new CANSparkMax(0, MotorType.kBrushless);
  }

  public void extend() {
    leftSpin.set(0.5);
    rightSpin.set(0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
