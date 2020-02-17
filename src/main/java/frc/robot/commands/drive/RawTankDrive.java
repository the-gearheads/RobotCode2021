/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RawTankDrive extends CommandBase {
  private DriveSubsystem drive;
  private XboxController controller;

  public RawTankDrive(DriveSubsystem drive, XboxController controller) {
    this.drive = drive;
    this.controller = controller;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    drive.controller.rawTankDrive(controller.getRawAxis(1), controller.getRawAxis(4));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
