/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import frc.robot.subsystems.Shooter;

public class ShootAll extends Shoot {
  private Shooter shooter;

  public ShootAll(Shooter shooter) {
    super(shooter);
    this.shooter = shooter;
  }

  @Override
  public boolean isFinished() {
    return (shooter.ballCount == 0);
  }
}
