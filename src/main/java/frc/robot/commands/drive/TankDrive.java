/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class TankDrive extends CommandBase {
  DriveSubsystem drive;

  public TankDrive(DriveSubsystem subsystem) {
    drive = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    drive.controller.rawTankDrive(RobotContainer.controller.getRawAxis(1), RobotContainer.controller.getRawAxis(4));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
