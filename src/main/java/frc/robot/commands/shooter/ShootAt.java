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
import frc.robot.util.Voltages;

public class ShootAt extends CommandBase {
  private Shooter shooter;
  private PIDController leftController;
  private PIDController rightController;
  private double rpm;

  public ShootAt(Shooter shooter, double rpm) {
    this.shooter = shooter;
    this.leftController = new PIDController(.05, 0, 0.008);
    this.rightController = new PIDController(.05, 0, 0.008);
    this.rpm = rpm;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    leftController.setSetpoint(rpm / 60);
    rightController.setSetpoint(rpm / 60);
  }

  @Override
  public void execute() {
    double left = leftController.calculate(shooter.getLeftVelocity() / 60);
    double right = rightController.calculate(shooter.getRightVelocity() / 60);
    left = MathUtil.clamp(left, -250, 250);
    right = MathUtil.clamp(right, -250, 250);
    shooter.shootVolts(rpm, new Voltages(left, right));
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    shooter.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
