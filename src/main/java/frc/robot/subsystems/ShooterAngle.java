/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.angle.HoldAngle;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterAngle extends SubsystemBase {

  private final CANSparkMax angleMotor;
  private final AnalogInput pot;
  private double angle; 

  public ShooterAngle() {

    angleMotor = new CANSparkMax(26, MotorType.kBrushless);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setInverted(true);

    pot = new AnalogInput(Constants.SHOOTER_POT);
    angle = getPosition();

    Logger.configureLoggingAndConfig(this, false);
  }

  public double getPosition() {
    return (pot.getVoltage() * -24.98) + 112.6;
  }

  public void setAngle(double angle) {
    this.angle = angle;
  }

  @Log
  public double getAngle() {
    return angle;
  }
  
  public boolean isLimited(double direction) {
    double sign = Math.signum(direction);
    if (sign == 1) {
      return angleMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
    } else {
      return angleMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
    }
  }

  public void turnAngle(double speed) {
    angleMotor.setVoltage(speed);
  }
}
