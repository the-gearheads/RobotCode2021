/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.angle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterAngle;

public class AngleCalibrate extends CommandBase {

  private final ShooterAngle angle;
  private int state;
  private double topVoltage;
  private double bottomVoltage;

  public AngleCalibrate(ShooterAngle angle) {
    this.angle = angle;
  }

  @Override
  public void initialize() {
    state = 0;
  }

  @Override
  public void execute() {
    if (state == 0) {
      if (angle.isLimited(1)) {
        state = 1;
        topVoltage = angle.getVoltage();
      } else {
        angle.turnAngle(1);
      }
    } else if (state == 1) {
      if (angle.isLimited(-1)) {
        state = 2;
        bottomVoltage = angle.getVoltage();
      } else {
        angle.turnAngle(-1);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    angle.setVolts(topVoltage, bottomVoltage);
  }

  @Override
  public boolean isFinished() {
    return (state == 2);
  }
}
