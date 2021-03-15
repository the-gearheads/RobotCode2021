/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.sound.midi.Receiver;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Deadband;
import frc.robot.util.Tuple;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class Intake extends SubsystemBase {

  private final CANSparkMax lExtension;
  private final CANSparkMax rExtension;

  private final CANSparkMax pft;
  private final CANSparkMax intake;

  private final CANEncoder lEncoder;
  private final CANEncoder rEncoder;
  private final CANEncoder intakeEncoder;

  @Log
  private double leftPos;
  @Log
  private double rightPos;
  @Log
  private double velocity;

  public Intake() {
    lExtension = new CANSparkMax(27, MotorType.kBrushless);
    rExtension = new CANSparkMax(6, MotorType.kBrushless);
    lExtension.setInverted(true);
    rExtension.setInverted(true);

    pft = new CANSparkMax(28, MotorType.kBrushless);
    pft.setIdleMode(IdleMode.kCoast);

    intake = new CANSparkMax(35, MotorType.kBrushless);
    intake.setIdleMode(IdleMode.kCoast);
    intakeEncoder = intake.getEncoder();

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

  public void pft(double speed) {
    pft.set(speed);
  }

  public void intake(double speed) {
    intake.setVoltage(speed);
  }

  public void intakePer(double per) {
    intake.set(per);
  }

  public void retract(double speed) {
    lExtension.set(-speed);
    rExtension.set(-speed);
  }

  public Tuple getPosition() {
    double left = lEncoder.getPosition(); // * ENCODER_CONSTANT;
    double right = rEncoder.getPosition(); // * ENCODER_CONSTANT;
    return new Tuple(left, right);
  }

  public Tuple getVelocity() {
    double left = lEncoder.getVelocity(); // * ENCODER_CONSTANT;
    double right = rEncoder.getVelocity(); // * ENCODER_CONSTANT;
    return new Tuple(left, right);
  }

  public void periodic() {
    Tuple pos = getPosition();
    leftPos = pos.left;
    rightPos = pos.right;
    Tuple vel = getVelocity();
    velocity = (vel.left + vel.right) / 2;
  }

  @Log
  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
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
    // lExtension.setIdleMode(IdleMode.kBrake);
    // rExtension.setIdleMode(IdleMode.kBrake);
  }

  public void setExtended() {
    lEncoder.setPosition(Constants.LEFT_DISTANCE);
    rEncoder.setPosition(Constants.RIGHT_DISTANCE);
  }
}
