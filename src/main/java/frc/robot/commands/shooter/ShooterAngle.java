/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Deadband;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterAngle extends CommandBase {
  private final Shooter shooter;
  @Log
  private double target;
  @Log
  private double error;

  public ShooterAngle(Shooter shooter) {
    Logger.configureLoggingAndConfig(this, false);
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = SmartDashboard.getNumber("shooterAngle", shooter.getAnglePosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.turnAngle(getSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.turnAngle(0);
  }

  public double getSpeed() {
    error = target - shooter.getAnglePosition();
    return Math.copySign(1, error);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooter.isLimited(getSpeed()) || (Math.abs(error) < 0.25));
  }
}