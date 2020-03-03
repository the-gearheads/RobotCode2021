/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class BlockedElevate extends CommandBase {
  private final Elevator elevator;
  private final Shooter shooter;

  public BlockedElevate(Elevator elevator, Shooter shooter) {
    this.elevator = elevator;
    this.shooter = shooter;
    addRequirements(elevator, shooter);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (shooter.bottomBlocked()) {
      elevator.lower(0.4);
    } else {
      elevator.lower(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.lower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
