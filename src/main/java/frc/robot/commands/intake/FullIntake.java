/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Intake;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class FullIntake extends CommandBase {
  private final Intake intake;
  @Log
  private final PIDController controller;
  private final SimpleMotorFeedforward feedforward;

  @Log
  private double debug0;
  @Log
  private double debug1;

  public FullIntake(Intake intake) {
    this.intake = intake;
    this.controller = new PIDController(1, 0, 0);
    feedforward = new SimpleMotorFeedforward(0.18, 0.0626, 0.00475);

    // Logger.configureLoggingAndConfig(this, false);
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    this.controller.setSetpoint(1000);
  }

  @Override
  public void execute() {
    intake.pft(.5);
    double ff = feedforward.calculate(controller.getSetpoint());
    double effort = controller.calculate(intake.getIntakeVelocity());
    effort = MathUtil.clamp(effort, -2, 2);
    debug0 = ff;
    debug1 = effort;
    intake.intake(ff + effort);
  }

  @Override
  public void end(boolean interrupted) {
    intake.pft(0);
    intake.intake(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
