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
  private double speed;

  public FullIntake(Intake intake) {
    this(intake, 1);
  }

  public FullIntake(Intake intake, double speed) {
    this.intake = intake;
    this.controller = new PIDController(0.01, 0, 0);
    feedforward = new SimpleMotorFeedforward(0.22, 0.00103, 0);
    this.speed = speed;

    //Logger.configureLoggingAndConfig(this, false);
    // addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // double effort = controller.calculate(intake.getIntakeVelocity());
    // double ff = feedforward.calculate(controller.getSetpoint());
    // effort = MathUtil.clamp(effort, 0, 12);
    // debug1 = effort;
    intake.pft(.8 * speed);
    // debug1 = intake.getIntakeVelocity();
    intake.intakePer(1.00 * speed);
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
