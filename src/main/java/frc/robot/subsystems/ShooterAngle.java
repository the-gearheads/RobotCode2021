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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.angle.HoldAngle;

public class ShooterAngle extends SubsystemBase {

  private final CANSparkMax angleMotor;
  private final AnalogInput pot;

  public ShooterAngle() {

    angleMotor = new CANSparkMax(26, MotorType.kBrushless);
    // angleMotor.setInverted(true);
    angleMotor.setIdleMode(IdleMode.kBrake);

    // pot = new AnalogPotentiometer(Constants.SHOOTER_POT, -131, 127.3);
    pot = new AnalogInput(Constants.SHOOTER_POT);

    SmartDashboard.putNumber("shooterAngle", getPosition());
    SmartDashboard.putBoolean("gotoAngle", false);

    setDefaultCommand(new HoldAngle(this));
  }

  public double getPosition() {
    return (pot.getVoltage() * -24.98) + 112.6;
  }

  public boolean isLimited() {
    return (angleMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get()
        || angleMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get());
  }

  public void turnAngle(double speed) {
    angleMotor.setVoltage(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
