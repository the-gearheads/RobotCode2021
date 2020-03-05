/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SpeedModifier extends CommandBase {
  private final DriveSubsystem drive;
  private final double multiplier;

  public SpeedModifier(DriveSubsystem drive, double multiplier) {
    this.drive = drive;
    this.multiplier = multiplier;
  }

  @Override
  public void initialize() {
    drive.controller.setMultiplier(multiplier);
  }

  @Override
  public void end(boolean interrupted) {
    drive.controller.setMultiplier(1);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
