/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Shooter;

public class ShootAndElevate extends CommandBase {
  private Shooter shooter; 
  private PIDController controller;
  private double rpm;
  /**
   * Creates a new elevator.
   */
  public ShootAndElevate(Shooter shooter, double rpm) {
    this.shooter = shooter;
    controller = new PIDController(1, 0, 0);
    controller.setSetpoint(rpm);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double effort = controller.calculate(shooter.getAvgVelocity());
    effort = MathUtil.clamp(effort, -rpm, rpm);
    shooter.shoot(effort);  
    shooter.elevate();
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    shooter.shoot(0);
    shooter.elevate(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
