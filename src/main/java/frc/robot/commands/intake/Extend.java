/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Intake;
import frc.robot.util.Deadband;
import frc.robot.util.Tuple;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class Extend extends CommandBase {
  private Intake intake;
  @Log
  private PIDController leftController;
  @Log
  private PIDController rightController;

  @Log
  private double MAX_VOLTS = 2;

  // TODO: replace with proper measurements
  private final double LEFT_DISTANCE = 0.75;
  private final double RIGHT_DISTANCE = 0.75;

  @Log
  private double SETPOINT = 0.1; // m/s

  public Extend(Intake intake) {
    this.intake = intake;
    leftController = new PIDController(2, 0, 0);
    rightController = new PIDController(2, 0, 0);

    leftController.setSetpoint(SETPOINT);
    rightController.setSetpoint(SETPOINT);

    Logger.configureLoggingAndConfig(this, false);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Tuple velocity = intake.getVelocity();
    double left = leftController.calculate(velocity.left);
    double right = rightController.calculate(velocity.right);

    // REMINDER: with 0 as the second parameter, it will only correct forward, not
    // backward!
    left = MathUtil.clamp(left, 0, MAX_VOLTS);
    right = MathUtil.clamp(right, 0, MAX_VOLTS);

    intake.extend(left, right);
  }

  @Override
  public void end(boolean interrupted) {
    intake.extend(0, 0);
  }

  @Override
  public boolean isFinished() {
    Tuple position = intake.getPosition();
    return (Deadband.get(position.left, LEFT_DISTANCE, 0.2) == 0)
        && (Deadband.get(position.right, RIGHT_DISTANCE, 0.2) == 0);
  }
}
