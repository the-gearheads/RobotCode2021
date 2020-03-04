/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arms;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class Release extends CommandBase {
  private final Arms arms;
  private double target;

  public Release(Arms arms) {
    this.arms = arms;
    addRequirements(arms);
  }

  @Override
  public void initialize() {
    target = arms.getPosition() + 40;
  }

  @Override
  public void execute() {
    arms.run(1);
  }

  @Override
  public void end(boolean interrupted) {
    arms.run(0);
  }

  @Override
  public boolean isFinished() {
    return (arms.getPosition() > target);
  }
}
