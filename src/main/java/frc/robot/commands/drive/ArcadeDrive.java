/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Deadband;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class ArcadeDrive extends CommandBase {
  DriveSubsystem drive;
  @Log
  double x;
  @Log
  double y;

  public ArcadeDrive(DriveSubsystem subsystem) {
    Logger.configureLoggingAndConfig(this, false);
    drive = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    // Speed multiplier should never be more than one, since it is multiplied by max
    // speed
    x = RobotContainer.controller.getRawAxis(4);
    y = RobotContainer.controller.getRawAxis(1); // TODO: Why does this have to be inverted?

    x = Deadband.getSmart(x, Constants.X_DEADBAND);
    y = Deadband.getSmart(y, Constants.Y_DEADBAND);

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
