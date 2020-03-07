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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Voltages;

public class Shoot extends CommandBase {
  private Shooter shooter;
  private PIDController leftController;
  private PIDController rightController;
  private double rpm;

  public Shoot(Shooter shooter) {
    this.shooter = shooter;
    this.leftController = new PIDController(.02, 0, 0.005);
    this.rightController = new PIDController(.02, 0, 0.005);
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    double range = Constants.RPM_MAX - Constants.RPM_MIN;
    rpm = (Math.abs(((RobotContainer.joystick.getRawAxis(2) - 1)) / 2)) * range + Constants.RPM_MIN;
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
