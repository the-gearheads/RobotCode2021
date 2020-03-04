/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Voltages;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase {
  private final CANSparkMax rShooter;
  private final CANSparkMax lShooter;

  private final CANEncoder lShooterEncoder;
  private final CANEncoder rShooterEncoder;

  private final AnalogInput irTop;
  private final AnalogInput irBottom;
  private boolean topPrimed;
  private boolean bottomPrimed;

  @Log
  public int ballCount;

  private final Supplier<Double> leftVelocity;
  private final Supplier<Double> rightVelocity;

  @Log
  private double shooterLeft;
  @Log
  private double shooterRight;
  private boolean newBall;

  private final SimpleMotorFeedforward leftFF;
  private final SimpleMotorFeedforward rightFF;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    lShooter = new CANSparkMax(7, MotorType.kBrushless);
    rShooter = new CANSparkMax(5, MotorType.kBrushless);
    rShooter.setInverted(true);

    lShooterEncoder = lShooter.getEncoder();
    rShooterEncoder = rShooter.getEncoder();

    leftVelocity = () -> lShooterEncoder.getVelocity();// * ENCODER_CONSTANT;
    rightVelocity = () -> rShooterEncoder.getVelocity();// * ENCODER_CONSTANT;

    leftFF = Constants.SHOOTER_FF_LEFT;
    rightFF = Constants.SHOOTER_FF_RIGHT;

    lShooter.setIdleMode(IdleMode.kBrake);
    rShooter.setIdleMode(IdleMode.kBrake);

    irTop = new AnalogInput(Constants.IR_TOP);
    irBottom = new AnalogInput(Constants.IR_BOTTOM);

    Logger.configureLoggingAndConfig(this, false);
  }

  public void shoot(double speed) {
    lShooter.set(speed);
    rShooter.set(speed);
  }

  public void shootVolts(double setpoint, Voltages voltages) {
    lShooter.setVoltage(leftFF.calculate(setpoint / 60) + voltages.left);
    rShooter.setVoltage(rightFF.calculate(setpoint / 60) + voltages.right);
  }

  public double getLeftVelocity() {
    return leftVelocity.get();
  }

  public double getRightVelocity() {
    return rightVelocity.get();
  }

  public boolean topBlocked() {
    return (irTop.getVoltage() < .1);
  }

  public boolean bottomBlocked() {
    return (irBottom.getVoltage() < .1);
  }

  public boolean getNewBall() {
    return newBall;
  }

  @Override
  public void periodic() {
    shooterLeft = leftVelocity.get();
    shooterRight = rightVelocity.get();

    if (bottomBlocked()) {
      if (bottomPrimed) {
        ballCount += 1;
        bottomPrimed = false;
        newBall = true;
      }
    } else {
      bottomPrimed = true;
    }

    if (topBlocked()) {
      if (topPrimed) {
        ballCount -= 1;
        topPrimed = false;
      }
    } else {
      topPrimed = true;
    }
  }
}