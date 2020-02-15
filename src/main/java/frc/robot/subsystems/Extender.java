/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {
  /**
   * Creates a new Extender.
   */
  private final CANSparkMax leftSpin;
  private final CANSparkMax rightSpin;
  private final WPI_TalonSRX intake;

  public Extender() {
    leftSpin = new CANSparkMax(22, MotorType.kBrushless);
    rightSpin = new CANSparkMax(0, MotorType.kBrushless);
    intake = new WPI_TalonSRX(0);
  }

  public void extend() {
    leftSpin.set(0.5);
    rightSpin.set(0.5);
    intake.set(0.8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
