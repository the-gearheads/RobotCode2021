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

public class ArcadeDrive extends CommandBase {
  DriveSubsystem drive;

  public ArcadeDrive(DriveSubsystem subsystem) {
    drive = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
  // Speed multiplier should never be more than one, since it is multiplied by max speed
    double x = RobotContainer.controller.getRawAxis(4);
    double y = RobotContainer.controller.getRawAxis(1); // TODO: Why does this have to be inverted?
    double xdeadband = 0.15;
    double ydeadband = 0.05;
    if (!(Math.abs(x) > xdeadband)) {x = 0;}
    if (!(Math.abs(y) > ydeadband)) {y = 0;}
    x = -(Constants.ROT_SPEED)*Math.pow(x,3);
    y *= -(Constants.THROTTLE_SPEED);
    
    ChassisSpeeds speeds = new ChassisSpeeds(y, 0, x);
    drive.controller.arcadeDrive(speeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
