/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.angle;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.util.Deadband;

public class HoldAngle extends CommandBase {
  private final ShooterAngle angle;
  private final PIDController up;
  private final PIDController down;

  private double target;

  public HoldAngle(ShooterAngle angle) {
    this.angle = angle;
    up = new PIDController(0.5, 0, 0);
    down = new PIDController(0.2, 0, 0);

    addRequirements(angle);
  }

  @Override
  public void initialize() {
    target = angle.getPosition();
    up.setSetpoint(target);
    down.setSetpoint(target);
  }

  @Override
  public void execute() {
    up.setSetpoint(angle.getSetpoint());
    down.setSetpoint(angle.getSetpoint());
    if (Deadband.get(angle.getPosition(), angle.getSetpoint(), 0.25) == 0) {
      angle.turnAngle(0);
      return;
    }

    double error = angle.getSetpoint() - angle.getPosition();
    double effort;
    if (Math.signum(error) == 1) {
      effort = up.calculate(angle.getPosition());
    } else {
      effort = down.calculate(angle.getPosition());
    }
    if (angle.isLimited(effort)) {
      angle.turnAngle(0);
      return;
    }

    effort = MathUtil.clamp(effort, -2, 2);
    angle.turnAngleVolts(effort);
  }

  @Override
  public void end(boolean interrupted) {
    angle.turnAngleVolts(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
