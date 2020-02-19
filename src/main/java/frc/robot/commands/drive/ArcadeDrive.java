/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Deadband;

public class ArcadeDrive extends CommandBase {
  private DriveSubsystem drive;

  public ArcadeDrive(DriveSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double x = RobotContainer.controller.getRawAxis(4);
    double y = RobotContainer.controller.getRawAxis(1);

    x = Deadband.getSmart(x, Constants.ROT_DEADBAND);
    y = Deadband.getSmart(y, Constants.THROTTLE_DEADBAND);

    x *= -Math.toRadians(Constants.ROT_SPEED);
    y *= -(Constants.THROTTLE_SPEED);

    ChassisSpeeds speeds = new ChassisSpeeds(y, 0, x);
    drive.controller.arcadeDrive(speeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
