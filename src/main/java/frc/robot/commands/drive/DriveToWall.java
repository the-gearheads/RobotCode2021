/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToWall extends CommandBase {
  private DriveSubsystem drive;
  private boolean isAtWall;

  public DriveToWall(DriveSubsystem drive) {
    this.drive = drive;
    this.isAtWall = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.controller.rawTankDrive(1 * Constants.SLOW_MULTIPLIER, 1 * Constants.SLOW_MULTIPLIER);
    if(drive.getLidarDistance() <= 10) {
      isAtWall = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return isAtWall;
  }
}
