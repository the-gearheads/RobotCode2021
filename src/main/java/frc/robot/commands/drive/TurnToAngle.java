/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Deadband;

public class TurnToAngle extends CommandBase {
  private final DriveSubsystem drive;
  private final double tolerance;
  private final boolean relative;
  private final PIDController controller;
  
  private double angle;

  public TurnToAngle(DriveSubsystem drive, double angle, double tolerance, boolean relative) {
    this.drive = drive;
    this.angle = angle;
    this.tolerance = tolerance;
    this.relative = relative;
    this.controller = new PIDController(Constants.ANGLE_P, 0, 0);
    addRequirements(drive);
  }

  public TurnToAngle(DriveSubsystem drive, double angle, double tolerance) {
    // Assume absolute
    this(drive, angle, tolerance, false);
  }

  @Override
  public void initialize() {
    if (relative) {
      angle += (drive.getAngle() % 360);
    }
  }

  @Override
  public void execute() {
    double error = angle - drive.getAngle();
    double speed = Math.toRadians(Math.copySign(Constants.ROT_SPEED, error));
    double effort = controller.calculate(error);
    drive.controller
        .arcadeDrive(new ChassisSpeeds(0.0, 0.0, speed + effort));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return (Deadband.get(drive.getAngle() % 360, tolerance) == 0);
  }
}
