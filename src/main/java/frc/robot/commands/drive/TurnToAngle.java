/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Deadband;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class TurnToAngle extends CommandBase {
  private final DriveSubsystem drive;
  private final PIDController controller;
  private final double offset;
  @Log
  private double effort;
  
  @Log
  private double angle;

  public TurnToAngle(DriveSubsystem drive) {
    this.drive = drive;
    this.controller = new PIDController(8, 0, 0);
    this.offset = Math.copySign(180, drive.getAngle())-drive.getAngle();
    addRequirements(drive);
    Logger.configureLoggingAndConfig(this, false);
  }

  @Override
  public void initialize() {
    angle = (NetworkTableInstance.getDefault().getTable("OpenSight").getEntry("camera").getDouble(0) + getAngle());
    controller.enableContinuousInput(0, 360);
    controller.setSetpoint(angle);
  }

  @Override
  public void execute() {
    effort = MathUtil.clamp(controller.calculate(getAngle()), -Constants.ROT_SPEED, Constants.ROT_SPEED);
    drive.controller
        .arcadeDrive(new ChassisSpeeds(0.0, 0.0, Math.toRadians(effort)));
  }

  @Override
  public void end(boolean interrupted) {
  }

  private double getAngle() {
    return (drive.getAngle()+offset) % 360;
  }

  @Override
  public boolean isFinished() {
    return (Deadband.get(getAngle(), angle, 0.5) == 0);
  }
}
