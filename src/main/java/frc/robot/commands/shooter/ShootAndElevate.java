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
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Shooter;

public class ShootAndElevate extends CommandBase {
  private Shooter shooter; 
  private PIDController leftController;
  private PIDController rightController;
  private double rpm;
  private double target;
  private double error;
  /**
   * Creates a new elevator.
   */
  public ShootAndElevate(Shooter shooter) {
    this.shooter = shooter;
    leftController = new PIDController(.05, 0, 0.008);
    rightController = new PIDController(.05, 0, 0.008);
    addRequirements(shooter);
    SmartDashboard.putNumber("rpm", 4000);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rpm = SmartDashboard.getNumber("rpm", 0);
    target = SmartDashboard.getNumber("shooterAngle", shooter.getAnglePosition());
    leftController.setSetpoint(rpm/60);
    rightController.setSetpoint(rpm/60);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = leftController.calculate(shooter.getLeftVelocity() / 60);
    double right = rightController.calculate(shooter.getRightVelocity() / 60);
    left = MathUtil.clamp(left, -250, 250);
    right = MathUtil.clamp(right, -250, 250);
    shooter.shootVolts(rpm, left, right);  
    shooter.elevate();
    shooter.turnAngle(getSpeed());
  }

  public double getSpeed() {
    error = target - shooter.getAnglePosition();
    return Math.copySign(.5, error);
  }

  // Called once the command ends or is interrupted.
  public void end(boolean interrupted) {
    shooter.shoot(0);
    shooter.elevate(0, 0);
    shooter.turnAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
