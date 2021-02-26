// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.util.Tuple;

public class RangeShoot extends CommandBase {

  // left = angle, right = shooter speed
  private final Tuple[] ranges = new Tuple[] { new Tuple(1, 1), new Tuple(2, 2), new Tuple(3, 3) };
  // maps volts to ranges
  private final double[] voltRanges = new double[] { 0, 1, 2 };

  private final Shooter shooter;
  private final ShooterAngle shooterAngle;

  private PIDController leftController;
  private PIDController rightController;

  private double angle;
  private double rpm;

  public RangeShoot(Shooter shooter, ShooterAngle shooterAngle) {
    this.shooter = shooter;
    this.shooterAngle = shooterAngle;
    this.leftController = new PIDController(Constants.SHOOTER_P, 0, Constants.SHOOTER_D);
    this.rightController = new PIDController(Constants.SHOOTER_P, 0, Constants.SHOOTER_D);
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    Tuple setting = getSpeed(shooter.getRange());
    angle = setting.left;
    rpm = setting.right;
    shooterAngle.setSetpoint(angle);
    leftController.setSetpoint(rpm / 60);
    rightController.setSetpoint(rpm / 60);
  }

  private Tuple getSpeed(double voltage) {
    Tuple setting = new Tuple(0, 0);
    int index = 0;
    for (double min : voltRanges) {
      if (voltage >= min) {
        setting = ranges[index];
      }
      index++;
    }
    return setting;
  }

  @Override
  public void execute() {
    double left = leftController.calculate(shooter.getLeftVelocity() / 60);
    double right = rightController.calculate(shooter.getRightVelocity() / 60);
    left = MathUtil.clamp(left, -250, 250);
    right = MathUtil.clamp(right, -250, 250);
    shooter.shootVolts(rpm, new Tuple(left, right));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
