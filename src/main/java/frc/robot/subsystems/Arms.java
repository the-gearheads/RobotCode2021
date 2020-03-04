/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Arms extends SubsystemBase {
  /**
   * Creates a new Extender.
   */
  private final CANSparkMax arm;
  private final CANEncoder encoder;

  @Log
  private double position;

  public Arms() {
    arm = new CANSparkMax(25, MotorType.kBrushless);
    arm.setInverted(true);
    encoder = arm.getEncoder();
  }

  public double getPosition() {
    return position;
  }

  public void run(double speed) {
    // NEVER RUN BACKWARD
    // speed = Math.abs(speed);
    arm.set(speed);
  }

  @Override
  public void periodic() {
    position = encoder.getPosition();
  }
}
