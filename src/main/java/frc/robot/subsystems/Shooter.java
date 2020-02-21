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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax rShooter;
  private final CANSparkMax lShooter;
  private final CANSparkMax angleMotor;

  private final CANEncoder lShooterEncoder;
  private final CANEncoder rShooterEncoder;

  private final CANSparkMax elevatorUpper;
  private final CANSparkMax elevatorLower;

  private final Supplier<Double> leftVelocity;
  private final Supplier<Double> rightVelocity;

  public final DoubleSupplier avgVelocity;

  private final SimpleMotorFeedforward feedforward;

  private final AnalogInput pot;


  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    lShooter = new CANSparkMax(7, MotorType.kBrushless);
    rShooter = new CANSparkMax(5, MotorType.kBrushless);
    rShooter.setInverted(true);

    angleMotor = new CANSparkMax(0, MotorType.kBrushless);

    lShooterEncoder = lShooter.getEncoder();
    rShooterEncoder = rShooter.getEncoder();

    leftVelocity = () -> lShooterEncoder.getVelocity() * Constants.SHOOTER_ENCODER_EPR;
    rightVelocity = () -> rShooterEncoder.getVelocity() * Constants.SHOOTER_ENCODER_EPR;
    avgVelocity = () -> getAvgVelocity();

    elevatorUpper = new CANSparkMax(15, MotorType.kBrushless);
    elevatorLower = new CANSparkMax(11, MotorType.kBrushless);
    elevatorLower.setInverted(true);

    feedforward = Constants.shooterFF;

    pot = new AnalogInput(Constants.SHOOTER_POT);

    lShooter.setIdleMode(IdleMode.kBrake);
    rShooter.setIdleMode(IdleMode.kBrake);
    elevatorUpper.setIdleMode(IdleMode.kBrake);
    elevatorLower.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("shootSpeed", .5);
    SmartDashboard.putNumber("upperSpeed", .6);
    SmartDashboard.putNumber("lowerSpeed", .3);
  }

  public void elevate(double upper, double lower) {
    elevatorUpper.set(upper);
    elevatorLower.set(lower);
  }

  public void shoot(double speed) {
    lShooter.set(speed);
    rShooter.set(speed);
  }

  public void shootVolts(double setpoint, double effort) {
    lShooter.setVoltage(feedforward.calculate(setpoint) + effort);
    rShooter.setVoltage(feedforward.calculate(setpoint) + effort);
  }

  public double getAvgVelocity() {
    return (leftVelocity.get() + rightVelocity.get()) / 2;
  }

  public double getAnglePosition() {
    return pot.getVoltage();
  }

  public void turnAngle(double speed) {
    angleMotor.set(speed);
  }
}
