/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SpeedModifier extends CommandBase {
  private final DriveSubsystem drive;
  private final Optional<Double> leftMult;
  private final Optional<Double> rightMult;
  private final Optional<Double> angleMult;

  public SpeedModifier(DriveSubsystem drive, Optional<Double> leftMult, Optional<Double> rightMult, Optional<Double> angleMult) {
    this.drive = drive;
    this.leftMult = leftMult;
    this.rightMult = rightMult;
    this.angleMult = angleMult;
  }

  @Override
  public void initialize() {
    if (leftMult.isPresent()) {
      drive.leftSpeedMultiplier = leftMult.get();
    }
    if (rightMult.isPresent()) {
      drive.rightSpeedMultiplier = rightMult.get();
    }
    if (angleMult.isPresent()) {
      drive.rotMultiplier = angleMult.get();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (leftMult.isPresent()) {
      drive.leftSpeedMultiplier = 1;
    }
    if (rightMult.isPresent()) {
      drive.rightSpeedMultiplier = 1;
    }
    if (angleMult.isPresent()) {
      drive.rotMultiplier = 1;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
