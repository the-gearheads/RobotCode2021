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
  private PIDController leftController;
  private PIDController rightController;
  @Log
  private double debug0;
  @Log
  private double debug1;
  @Log
  private boolean debug2;
  private final double L_SETPOINT = 30.64;
  private final double R_SETPOINT = 30.64;

  /**
   * Creates a new Intake.
   */
  public Extend(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    leftController = new PIDController(1.2, 0, 0);
    rightController = new PIDController(1, 0, 0);
    leftController.setSetpoint(L_SETPOINT);
    rightController.setSetpoint(R_SETPOINT);
    Logger.configureLoggingAndConfig(this, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double axis = RobotContainer.controller.getRawAxis(1) * 0.5;
    // intake.extend(axis);
    Tuple position = intake.getPosition();
    double left = leftController.calculate(position.left);
    left = MathUtil.clamp(left, 0, 2);
    double right = rightController.calculate(position.right);
    right = MathUtil.clamp(right, 0, 2);

    debug0 = left;
    debug1 = right;
    intake.extend(left, right);
    // intake.setCoast();
  }

  // @Config
  // public void setLeft(double kP) {
  // leftController.setP(kP);
  // }

  // @Config
  // public void setRight(double kP) {
  // rightController.setP(kP);
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.extend(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Tuple position = intake.getPosition();
    return Deadband.get(position.left, L_SETPOINT, .3) == 0 && Deadband.get(position.right, R_SETPOINT, .3) == 0;
  }
}
