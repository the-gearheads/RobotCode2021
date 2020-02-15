/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private final CANSparkMax extension;
  private final CANEncoder encoder;
  private final WPI_TalonSRX intake;

  public Intake() {
    extension = new CANSparkMax(0, MotorType.kBrushless);
    intake = new WPI_TalonSRX(0);
    encoder = extension.getEncoder();
  }

  public void extend(double speed) {
    extension.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake(double speed) {
    intake.set(speed);
  }

  public void retract(double speed) {
    extension.set(-speed);
  }

  public double getPosition() {
    return encoder.getPosition();
  }
}
