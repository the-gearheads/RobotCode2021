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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.angle.HoldAngle;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterAngle extends SubsystemBase {

  private final NetworkTableEntry top;
  private final NetworkTableEntry bottom;

  @Log
  private double topVolts;
  @Log
  private double bottomVolts;

  private final CANSparkMax angleMotor;
  private final AnalogInput pot;
  @Log
  private double setpoint;

  public ShooterAngle() {

    angleMotor = new CANSparkMax(26, MotorType.kBrushless);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setInverted(true);

    pot = new AnalogInput(Constants.SHOOTER_POT);

    top = NetworkTableInstance.getDefault().getTable("Config/ShooterAngle").getEntry("Top");
    bottom = NetworkTableInstance.getDefault().getTable("Config/ShooterAngle").getEntry("Bottom");
    top.setPersistent();
    bottom.setPersistent();

    updateVolts();
    setpoint = getPosition();

    setDefaultCommand(new HoldAngle(this));
    Logger.configureLoggingAndConfig(this, false);
  }

  @Log
  public double getPosition() {
    double p = (getVoltage() - bottomVolts) / (topVolts - bottomVolts);
    return Constants.SHOOTER_ANGLE_MIN + (p * (Constants.SHOOTER_ANGLE_MAX - Constants.SHOOTER_ANGLE_MIN));
  }

  public void updateVolts() {
    topVolts = top.getDouble(0);
    bottomVolts = bottom.getDouble(5);
  }

  public void setVolts(double top, double bottom) {
    this.top.setDouble(top);
    this.bottom.setDouble(bottom);
    topVolts = top;
    bottomVolts = bottom;
  }

  public double getVoltage() {
    return pot.getVoltage();
  }

  @Config
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public double getSetpoint() {
    return setpoint;
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
    angleMotor.set(speed);
  }

  public void turnAngleVolts(double speed) {
    angleMotor.setVoltage(speed);
  }
}
