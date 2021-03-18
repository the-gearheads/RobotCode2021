// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Ramsete;

public class Pathweaver extends CommandBase {
  private final DriveSubsystem drive;
  private final String name;
  private Ramsete ramsete;

  public Pathweaver(DriveSubsystem drive, String name) {
    this.drive = drive;
    this.name = name;
  }

  @Override
  public void initialize() {
    try {
      Path path = Filesystem.getDeployDirectory().toPath().resolve("output/" + name + ".wpilib.json"); 
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);
      
      drive.odometry.resetPosition(trajectory.getInitialPose(), trajectory.getInitialPose().getRotation());
      ramsete = new Ramsete(trajectory, new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA), drive);
      ramsete.schedule();
      
    } catch (MalformedSplineException | IOException e) {
      DriverStation.reportError("Failed to generate Pathweaver trajectory", e.getStackTrace());
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    ramsete.cancel();
  }

  @Override
  public boolean isFinished() {
    return ramsete.isFinished();
  }
}
