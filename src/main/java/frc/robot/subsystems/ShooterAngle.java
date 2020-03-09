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
import io.github.oblarg.oblog.annotations.Log;

public class ShooterAngle extends SubsystemBase {

  private final NetworkTableEntry top;
  private final NetworkTableEntry bottom;

  private double topVolts;
  private double bottomVolts;

  private final CANSparkMax angleMotor;
  private final AnalogInput pot;
  private double setpoint;

  public ShooterAngle() {

    angleMotor = new CANSparkMax(26, MotorType.kBrushless);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setInverted(true);

    pot = new AnalogInput(Constants.SHOOTER_POT);
    setpoint = getPosition();

    top = NetworkTableInstance.getDefault().getTable("Config/ShooterAngle").getEntry("Slope");
    bottom = NetworkTableInstance.getDefault().getTable("Config/ShooterAngle").getEntry("Offset");
    top.setPersistent();
    bottom.setPersistent();
    updateVolts();

    setDefaultCommand(new HoldAngle(this));
    Logger.configureLoggingAndConfig(this, false);
  }

  public double getPosition() {
    return bottomVolts + (pot.getVoltage() / 5) * (topVolts - bottomVolts);
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

  public void setSetpoint(double angle) {
    this.setpoint = angle;
  }

  @Log
  public double getAngle() {
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
    angleMotor.setVoltage(speed);
  }
}
