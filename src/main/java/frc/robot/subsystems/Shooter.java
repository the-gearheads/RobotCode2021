/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase {
  private final CANSparkMax rShooter;
  private final CANSparkMax lShooter;

  private final CANSparkMax elevatorUpper;
  private final CANSparkMax elevatorLower;
  private final CANEncoder upperEnc;
  private final CANEncoder lowerEnc;

  private final AnalogInput irTop;
  private final AnalogInput irBottom;
  private boolean topPrimed;
  private boolean bottomPrimed;

  @Log
  public int ballCount;
  @Log
  private double shootSpeed;
  @Log
  private double upperSpeed;
  @Log
  private double lowerSpeed;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    lShooter = new CANSparkMax(7, MotorType.kBrushless);
    rShooter = new CANSparkMax(5, MotorType.kBrushless);

    elevatorUpper = new CANSparkMax(15, MotorType.kBrushless);
    elevatorLower = new CANSparkMax(11, MotorType.kBrushless);
    upperEnc = elevatorUpper.getEncoder();
    lowerEnc = elevatorLower.getEncoder();

    lShooter.setIdleMode(IdleMode.kBrake);
    rShooter.setIdleMode(IdleMode.kBrake);

    elevatorUpper.setIdleMode(IdleMode.kBrake);
    elevatorLower.setIdleMode(IdleMode.kBrake);

    irTop = new AnalogInput(Constants.IR_TOP);
    irBottom = new AnalogInput(Constants.IR_BOTTOM);

    Logger.configureLoggingAndConfig(this, false);
  }

  public void elevator() {
    elevator(upperSpeed, lowerSpeed);
  }

  public void elevator(double upper, double lower) {
    elevatorUpper.set(upper);
    elevatorLower.set(lower);
  }

  public void shot() {
    shot(shootSpeed);
  }
  public void shot(double speed) {
    lShooter.set(speed);
    rShooter.set(speed);
  }

  public double getElevatorVelocity() {
    return (upperEnc.getVelocity() + lowerEnc.getVelocity()) / 2;
  }

  @Override
  public void periodic() {
    if (irBottom.getVoltage() == 0) {
      if (bottomPrimed) {
        ballCount += Math.signum(getElevatorVelocity());
        bottomPrimed = false;
      }
    } else {
      bottomPrimed = true;
    }

    if (irTop.getVoltage() == 0) {
      if (topPrimed) {
        ballCount += 1;
        topPrimed = true;
      }
    } else {
      topPrimed = false;
    }
  }
}
