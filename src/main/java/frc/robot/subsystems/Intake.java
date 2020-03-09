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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.intake.RunIntake;
import frc.robot.util.Deadband;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private final CANSparkMax lExtension;
  private final CANSparkMax rExtension;
  private final CANSparkMax pft;

  private final WPI_TalonSRX intake;

  private final CANEncoder lEncoder;
  private final CANEncoder rEncoder;
  @Log
  private double left;
  @Log
  private double right;

  public Intake() {
    lExtension = new CANSparkMax(27, MotorType.kBrushless);
    rExtension = new CANSparkMax(6, MotorType.kBrushless);
    lExtension.setInverted(true);
    rExtension.setInverted(true);
    rExtension.setIdleMode(IdleMode.kCoast);

    pft = new CANSparkMax(28, MotorType.kBrushless);
    intake = new WPI_TalonSRX(35);
    lEncoder = lExtension.getEncoder();
    rEncoder = rExtension.getEncoder();
    lEncoder.setPosition(0);
    rEncoder.setPosition(0);

    Logger.configureLoggingAndConfig(this, false);
  }

  public void extend(double left, double right) {
    lExtension.setVoltage(left);
    rExtension.setVoltage(right);
  }

  @Override
  public void periodic() {
    left = lEncoder.getPosition();
    right = rEncoder.getPosition();
  }

  public void pft(double speed) {
    pft.set(speed);
  }

  public void intake(double speed) {
    intake.set(speed);
  }

  public void retract(double speed) {
    lExtension.set(-speed);
    rExtension.set(-speed);
  }

  public double getLPosition() {
    return lEncoder.getPosition();
  }

  public double getRPosition() {
    return rEncoder.getPosition();
  }

  @Log
  public boolean isJammed() {
    return (Deadband.get(lExtension.getAppliedOutput(), 0, 1.5) != 0)
        || (Deadband.get(rExtension.getAppliedOutput(), 0, 1.5) != 0);
  }

  public void setCoast() {
    lExtension.setIdleMode(IdleMode.kCoast);
    rExtension.setIdleMode(IdleMode.kCoast);
  }

  public void setBrake() {
    lExtension.setIdleMode(IdleMode.kBrake);
    rExtension.setIdleMode(IdleMode.kBrake);
  }
}
