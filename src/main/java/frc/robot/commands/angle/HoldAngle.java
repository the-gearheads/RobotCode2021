/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.angle;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.util.Deadband;

public class HoldAngle extends CommandBase {
  private final ShooterAngle angle;
  private final PIDController controller;

  private double target;

  public HoldAngle(ShooterAngle angle) {
    this.angle = angle;

    controller = new PIDController(0.075, 0, 0);
    // controller.setTolerance(0);

    addRequirements(angle);
  }

  @Override
  public void initialize() {
    target = angle.getPosition();
    controller.setSetpoint(target);
  }

  @Override
  public void execute() {
    controller.setSetpoint(angle.getAngle());
    if (Deadband.get(angle.getPosition(), angle.getAngle(), 5) == 0) {
      SmartDashboard.putBoolean("atsetpoint", true);
      return;
    }
    SmartDashboard.putBoolean("atsetpoint", false);

    double effort = controller.calculate(angle.getPosition());
    if (angle.isLimited(effort)) {
      SmartDashboard.putBoolean("islimited", true);
      return;
    }
    SmartDashboard.putBoolean("islimited", false);

    effort = MathUtil.clamp(effort, -10, 10);
    angle.turnAngle(effort);
  }

  @Override
  public void end(boolean interrupted) {
    angle.turnAngle(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
