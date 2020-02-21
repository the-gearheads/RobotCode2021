/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Deadband;

public class ShooterAngle extends CommandBase {
  Shooter shooter;
  double target;

  public ShooterAngle(Shooter shooter, double target) {
    this.shooter = shooter;
    this.target = target;
    addRequirements(shooter);
  }

  public ShooterAngle(Shooter shooter) {
    this.shooter = shooter;
    this.target = SmartDashboard.getNumber("shooterAngle", shooter.getAnglePosition());
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double direction = target - shooter.getAnglePosition();
    shooter.turnAngle(Math.copySign(0.2, direction));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.turnAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Deadband.get(shooter.getAnglePosition(), 0.5) == 0);
  }
}