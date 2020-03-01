/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase {
  private final CANSparkMax rShooter;
  private final CANSparkMax lShooter;
  private final CANSparkMax angleMotor;

  private final CANEncoder lShooterEncoder;
  private final CANEncoder rShooterEncoder;

  private final CANSparkMax elevatorUpper;
  private final CANSparkMax elevatorLower;
  private final CANEncoder upperEnc;
  private final CANEncoder lowerEnc;

  // private final AnalogInput irTop;
  private final AnalogInput irBottom;
  private boolean topPrimed;
  private boolean bottomPrimed;

  @Log
  public int ballCount;
  private double shootSpeed = 0.5;
  private double upperSpeed = 0.6;
  private double lowerSpeed = 0.4;

  private final Supplier<Double> leftVelocity;
  private final Supplier<Double> rightVelocity;

  @Log
  private double shooterLeft;
  @Log
  private double shooterRight;

  private final SimpleMotorFeedforward leftFF;
  private final SimpleMotorFeedforward rightFF;

  @Log
  private final AnalogInput pot;
  @Log
  private double volt;
  @Log
  private double anglePosition;
  @Log
  private double debug1;
  @Log
  private double debug2;
  @Log
  private double debug3;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    lShooter = new CANSparkMax(7, MotorType.kBrushless);
    rShooter = new CANSparkMax(5, MotorType.kBrushless);
    rShooter.setInverted(true);

    angleMotor = new CANSparkMax(26, MotorType.kBrushless);
    angleMotor.setInverted(true);
    angleMotor.setIdleMode(IdleMode.kBrake);

    lShooterEncoder = lShooter.getEncoder();
    rShooterEncoder = rShooter.getEncoder();

    leftVelocity = () -> lShooterEncoder.getVelocity();// * ENCODER_CONSTANT;
    rightVelocity = () -> rShooterEncoder.getVelocity();// * ENCODER_CONSTANT;

    elevatorUpper = new CANSparkMax(15, MotorType.kBrushless);
    elevatorLower = new CANSparkMax(11, MotorType.kBrushless);
    upperEnc = elevatorUpper.getEncoder();
    lowerEnc = elevatorLower.getEncoder();
    elevatorLower.setInverted(true);

    leftFF = Constants.SHOOTER_FF_LEFT;
    rightFF = Constants.SHOOTER_FF_RIGHT;

    // pot = new AnalogPotentiometer(Constants.SHOOTER_POT, -131, 127.3);
    pot = new AnalogInput(Constants.SHOOTER_POT);

    lShooter.setIdleMode(IdleMode.kBrake);
    rShooter.setIdleMode(IdleMode.kBrake);

    elevatorUpper.setIdleMode(IdleMode.kBrake);
    elevatorLower.setIdleMode(IdleMode.kBrake);

    // irTop = new AnalogInput(Constants.IR_TOP);
    irBottom = new AnalogInput(Constants.IR_BOTTOM);

    Logger.configureLoggingAndConfig(this, false);
  }

  @Config
  /**
   * @param ballCount the ballCount to set
   */
  public void setBallCount(int ballCount) {
    this.ballCount = ballCount;
  }

  /**
   * @param shootSpeed the shootSpeed to set
   */
  @Config
  public void setShootSpeed(double shootSpeed) {
    this.shootSpeed = shootSpeed;
  }

  /**
   * @param lowerSpeed the lowerSpeed to set
   */
  @Config
  public void setLowerSpeed(double lowerSpeed) {
    this.lowerSpeed = lowerSpeed;
  }

  /**
   * @param upperSpeed the upperSpeed to set
   */
  @Config
  public void setUpperSpeed(double upperSpeed) {
    this.upperSpeed = upperSpeed;
  }

  @Override
  public void periodic() {
    if (irBottom.getVoltage() < .1) {
      if (bottomPrimed) {
        ballCount += 1;
        bottomPrimed = false;
      }
    } else {
      bottomPrimed = true;
    }
    shooterLeft = leftVelocity.get();
    shooterRight = rightVelocity.get();
    anglePosition = getAnglePosition();
    debug1 = lShooter.getOutputCurrent();
    // debug1 = pot.getVoltage();
    // debug2 = RobotController.getVoltage5V();
    // if (irTop.getVoltage() == 0) {
    // if (topPrimed) {
    // ballCount += 1;
    // topPrimed = true;
    // }
    // } else {
    // topPrimed = false;
    // }

  }

  public double getElevatorVelocity() {
    return (upperEnc.getVelocity() + lowerEnc.getVelocity()) / 2;
  }

  public void elevate() {
    elevate(0.4, 0.4);
  }

  public void elevate(double upper, double lower) {
    elevatorUpper.set(upper);
    elevatorLower.set(lower);
  }

  public void shoot() {
    shoot(shootSpeed);
  }

  public void shoot(double speed) {
    lShooter.setVoltage(speed);
    rShooter.setVoltage(speed);
  }

  // TODO: these are stupid params
  public void shootVolts(double setpoint, double effortLeft, double effortRight) {
    lShooter.setVoltage(leftFF.calculate(setpoint / 60) + effortLeft);
    rShooter.setVoltage(rightFF.calculate(setpoint / 60) + effortRight);
  }

  /**
   * @return the leftVelocity
   */
  public double getLeftVelocity() {
    return leftVelocity.get();
  }

  /**
   * @return the rightVelocity
   */
  public double getRightVelocity() {
    return rightVelocity.get();
  }

  public double getAnglePosition() {
    return (pot.getVoltage() * -24.98) + 112.6;
  }

  public boolean isLimited(double direction) {
    if (Math.signum(direction) == 1) {
      return angleMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
    } else {
      return angleMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
    }
  }

  public void turnAngle(double speed) {
    angleMotor.setVoltage(speed);
  }
}
