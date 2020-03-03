/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Shooter;

public class ShootAll extends Shoot {
  private final Shooter shooter;
  private final Timer timer;

  public ShootAll(Shooter shooter) {
    super(shooter);
    this.shooter = shooter;

    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.reset();
    super.initialize();
  }

  @Override
  public boolean isFinished() {
    if (shooter.ballCount == 0) {
      timer.start();
    }
    return timer.hasPeriodPassed(2);
  }
}
