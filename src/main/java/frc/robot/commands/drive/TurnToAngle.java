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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Deadband;

public class TurnToAngle extends CommandBase {
  private final DriveSubsystem drive;
  private double angle;
  private final double tolerance;
  private final boolean relative;

  public TurnToAngle(DriveSubsystem drive, double angle, double tolerance, boolean relative) {
    this.drive = drive;
    this.angle = angle;
    this.tolerance = tolerance;
    this.relative = relative;
    addRequirements(drive);
  }

  public TurnToAngle(DriveSubsystem drive, double angle, double tolerance) {
    // Assume absolute
    this(drive, angle, tolerance, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (relative) {
      angle += drive.getAngle();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = angle - drive.getAngle();
    drive.controller
        .arcadeDrive(new ChassisSpeeds(0.0, 0.0, Math.toRadians(Math.copySign(Constants.ROT_SPEED, error))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Deadband.get(drive.getAngle() % 360, tolerance) == 0);
  }
}
