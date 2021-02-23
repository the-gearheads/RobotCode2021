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

public class Retract extends CommandBase {
  private Intake intake;
  @Log
  private PIDController leftController;
  @Log
  private PIDController rightController;
  private final double SETPOINT = .1;

  @Log
  private double debug0;
  @Log
  private double debug1;

  /**
   * Creates a new Intake.
   */
  public Retract(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    leftController = new PIDController(.95, 0, 0);
    rightController = new PIDController(.98, 0, 0);
    leftController.setSetpoint(SETPOINT);
    rightController.setSetpoint(SETPOINT);
    Logger.configureLoggingAndConfig(this, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Tuple position = intake.getPosition();
    double left = leftController.calculate(position.left);
    left = MathUtil.clamp(left, -8, 0);
    double right = rightController.calculate(position.right);
    right = MathUtil.clamp(right, -8, 0);

    debug0 = left;
    debug1 = right;

    // intake.extend(-2, -2);
    intake.extend(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.extend(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Tuple position = intake.getPosition();
    return Deadband.get(position.left, SETPOINT, .4) == 0 && Deadband.get(position.right, SETPOINT, .4) == 0;
  }
}
