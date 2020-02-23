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

public class ShooterAngle extends CommandBase {
  private final Shooter shooter;
  private final PIDController controller;
  

  public ShooterAngle(Shooter shooter, double target) {
    this.shooter = shooter;
    controller = new PIDController(Constants.SHOOTER_ANGLE_P, 0, Constants.SHOOTER_ANGLE_D);
    controller.setSetpoint(target);
    addRequirements(shooter);
  }

  public ShooterAngle(Shooter shooter) {
    this(shooter, SmartDashboard.getNumber("shooterAngle", shooter.getAnglePosition()));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double effort = controller.calculate(shooter.getAnglePosition());
    shooter.turnAngle(effort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.turnAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Deadband.get(shooter.getAnglePosition(), 2) == 0);
  }
}