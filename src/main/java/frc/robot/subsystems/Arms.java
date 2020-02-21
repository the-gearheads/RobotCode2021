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

public class Arms extends SubsystemBase {
  /**
   * Creates a new Extender.
   */
  private final CANSparkMax leftArm;
  private final CANSparkMax rightArm;

  public Arms() {
    leftArm = new CANSparkMax(0, MotorType.kBrushless);
    rightArm = new CANSparkMax(0, MotorType.kBrushless);
  }

  public void extend() {
    leftArm.set(0.5);
    rightArm.set(0.5);
  }

  public void retract() {
    leftArm.set(-0.5);
    rightArm.set(-0.5);
  }

  public void zero() {
    leftArm.set(0);
    rightArm.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
