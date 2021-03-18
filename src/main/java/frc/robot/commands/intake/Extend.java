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
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.util.Deadband;
import frc.robot.util.Tuple;
import io.github.oblarg.oblog.annotations.Log;

public class Extend extends CommandBase {
  private Intake intake;

  @Log
  private PIDController leftController;
  @Log
  private PIDController rightController;

  private final double MAX_VOLTS = 8;

  private double errorLeft;
  private double errorRight;

  public Extend(Intake intake) {
    this.intake = intake;
    leftController = new PIDController(1, 0, 0);
    rightController = new PIDController(1, 0, 0);

    leftController.setSetpoint(Constants.LEFT_DISTANCE);
    rightController.setSetpoint(Constants.RIGHT_DISTANCE);

    //Logger.configureLoggingAndConfig(this, false);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Tuple position = intake.getPosition();

    double left = leftController.calculate(position.left);
    double right = rightController.calculate(position.right);

    errorLeft = Math.abs(position.left - Constants.LEFT_DISTANCE);
    errorRight = Math.abs(position.right - Constants.RIGHT_DISTANCE);

    boolean leftGoal = Deadband.get(errorLeft, 0, 0.2) == 0;
    boolean rightGoal = Deadband.get(errorRight, 0, 0.2) == 0;

    left = MathUtil.clamp(left, -2, leftGoal ? 0 : MAX_VOLTS);
    right = MathUtil.clamp(right, -2, rightGoal ? 0 : MAX_VOLTS);

    intake.extend(left, right);
  }

  @Override
  public void end(boolean interrupted) {
    intake.extend(0, 0);
  }

  @Override
  public boolean isFinished() {
    return Deadband.get(errorLeft + errorRight, 0, 0.5) == 0;
  }
}
