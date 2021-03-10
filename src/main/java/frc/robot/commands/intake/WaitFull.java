// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class WaitFull extends CommandBase {

  private final Shooter shooter;
  private final int ballsToShoot;
  private int decrements;

  public WaitFull(Shooter shooter, int ballsToShoot) {
    this.shooter = shooter;
    this.ballsToShoot = ballsToShoot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getDecremented()) {
      decrements += 1;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return decrements >= ballsToShoot;
  }
}
