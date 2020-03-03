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

public class HoldAngle extends CommandBase {
  private final ShooterAngle angle;
  private final PIDController controller;

  private double target;

  public HoldAngle(ShooterAngle angle) {
    this.angle = angle;

    target = angle.getPosition();
    controller = new PIDController(.4, 0, 0);
    controller.setSetpoint(target);
    controller.setTolerance(1);

    addRequirements(angle);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (SmartDashboard.getBoolean("gotoAngle", false)) {
      target = SmartDashboard.getNumber("shooterAngle", angle.getPosition());
      controller.setSetpoint(target);
      if (angle.isLimited()) {
        pause();
      }
      double effort = controller.calculate(angle.getPosition());
      effort = MathUtil.clamp(effort, -12, 12);
      angle.turnAngle(effort);
    }
  }

  public void pause() {
    controller.setSetpoint(angle.getPosition());
    SmartDashboard.putNumber("shooterAngle", angle.getPosition());
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
