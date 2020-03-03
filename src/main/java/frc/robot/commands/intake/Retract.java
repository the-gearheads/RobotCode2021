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

public class Retract extends CommandBase {
  private Intake intake;
  @Log
  private PIDController leftController;
  @Log
  private PIDController rightController;
  private final double SETPOINT = .8;

  /**
   * Creates a new Intake.
   */
  public Retract(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.intake = intake;
    leftController = new PIDController(.85, 0, 0);
    rightController = new PIDController(.88, 0, 0);
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
    double left = leftController.calculate(intake.getLPosition());
    left = MathUtil.clamp(left, -9.5, 0);
    double right = rightController.calculate(intake.getRPosition());
    right = MathUtil.clamp(right, -9.5, 0);

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
    if (intake.isJammed()) {
      intake.setCoast();
    }
    boolean exit = (intake.isJammed())
        || (Deadband.get(intake.getLPosition(), SETPOINT, .2) == 0)
            && (Deadband.get(intake.getRPosition(), SETPOINT, .2) == 0)
        || ((intake.getLPosition() < 0) || (intake.getRPosition() < 0));
    if (exit && (!intake.isJammed())) {
      intake.setBrake();
    }
    return exit;
  }
}
