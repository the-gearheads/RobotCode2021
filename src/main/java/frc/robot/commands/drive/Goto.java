// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Collections;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Ramsete;

public class Goto extends CommandBase {
  private final DriveSubsystem drive;
  private final Pose2d pose;
  private Ramsete ramsete;
  private boolean reversed;
  private NetworkTableInstance inst; 
  private NetworkTable table;
  private NetworkTableEntry xEntry;
  private NetworkTableEntry yEntry;
  private NetworkTableEntry isFollowing;
  private Timer timer = new Timer();
  private Trajectory trajectory;

  public Goto(DriveSubsystem drive, Pose2d pose, boolean reversed) {
    this.drive = drive;
    this.pose = pose;
    this.reversed = reversed;
    this.inst = NetworkTableInstance.getDefault();
    this.table = inst.getTable("Live_Dashboard");
    this.xEntry = this.table.getEntry("poseX");
    this.yEntry = this.table.getEntry("poseY");
    this.isFollowing = this.table.getEntry("isFollowingPath");
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        Constants.leftFF, drive.kinematics, 10);
    final TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_VELOCITY, Constants.MAX_ACCEL)
        .setKinematics(drive.kinematics).addConstraint(autoVoltageConstraint).setReversed(reversed);

    try {
      trajectory = TrajectoryGenerator.generateTrajectory(drive.getPose(), Collections.emptyList(), pose,
        config);
      
      ramsete = new Ramsete(trajectory, new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA), drive);
      ramsete.schedule();
      
      this.xEntry.setNumber(pose.getX());
      this.yEntry.setNumber(pose.getY());
      this.isFollowing.setBoolean(true);
    } catch (MalformedSplineException e) {
      DriverStation.reportError("Failed to generate routeToOrigin trajectory", e.getStackTrace());
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
    return ramsete.isFinished() || timer.hasElapsed(trajectory.getTotalTimeSeconds() + .5);
  }
}
