/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.util.Deadband;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Extend extends CommandBase {
  private Intake intake;
  @Log
  private PIDController leftController;
  @Log
  private PIDController rightController;
  private final double SETPOINT = 32.5;
  @Log
  private boolean debug0;
  @Log
  private boolean debug1;
  @Log
  private boolean debug2;

  /**
   * Creates a new Intake.
   */
  public Extend(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    leftController = new PIDController(.75, 0, 0);
    rightController = new PIDController(.75, 0, 0);
    leftController.setSetpoint(33);
    rightController.setSetpoint(33.2);
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
    double left = leftController.calculate(intake.getLPosition());
    left = MathUtil.clamp(left, 0, 9.5);
    double right = rightController.calculate(intake.getRPosition());
    right = MathUtil.clamp(right, 0, 9.5);

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
    if (intake.getLPosition() == 0 || intake.getRPosition() == 0) {
      return false; // rock solid logic baby
    }
    if (intake.isJammed()) {
      intake.setCoast();
    }
    return (intake.isJammed())
        || (Deadband.get(intake.getLPosition(), 32.6, 0.2) == 0)
            && (Deadband.get(intake.getRPosition(), 32.8, 0.2) == 0)
        || ((intake.getLPosition() >= 32.85) || (intake.getRPosition() >= 33.09));
  }
}
