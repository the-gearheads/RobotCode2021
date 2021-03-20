// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.misc.Rumble;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Deadband;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class StartTimer extends CommandBase {
  private final DriveSubsystem drive;
  private Timer timer = new Timer();
  @Log
  private int state = 0;
  @Log
  private double distance = 0;

  public StartTimer(DriveSubsystem drive) {
    this.drive = drive;
    Logger.configureLoggingAndConfig(this, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.odometry.resetPosition(new Pose2d(0,0, new Rotation2d(0)), new Rotation2d(0));
    drive.resetEncoders();
    timer.reset();
    state = 0;
  }

  @Log
  public double getTime() {
    return timer.get();
  }

  @Override
  public void execute() {
    double x = drive.odometry.getPoseMeters().getX();
    double y = drive.odometry.getPoseMeters().getY();
    distance = Math.abs(x) + Math.abs(y);
    if (state == 0 && distance > 0.25) { // start timer if at least 1m offset
      state++;
      timer.start();
    } else if (state == 1 && distance < 0.2) { // stop timer if within 0.5m of origin
      state++;
      timer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    (new Rumble(RobotContainer.controller).withTimeout(1)).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state >= 2;
  }
}
