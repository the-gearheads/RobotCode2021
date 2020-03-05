/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class WinchHold extends CommandBase {
  private final Arms arms;
  private final double direction;

  public WinchHold(Arms arms, double direction) {
    this.arms = arms;
    this.direction = direction;
    addRequirements(arms);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    arms.run(0.3*direction);
  }

  @Override
  public void end(boolean interrupted) {
    arms.run(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
