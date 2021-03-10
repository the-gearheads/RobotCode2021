// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.commands.drive.Goto;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.intake.Extend;
import frc.robot.commands.intake.FullIntake;
import frc.robot.commands.intake.WaitFull;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAll;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AccuracyChallenge extends ParallelCommandGroup {
  /** Creates a new AccuracyChallenge. */
  public AccuracyChallenge(DriveSubsystem drive, Shooter shooter, Intake intake, Elevator elevator, double feet) {
    addCommands(
      new Elevate(elevator), 
      //new Extend(intake), 
      new FullIntake(intake),
      new SequentialCommandGroup(
        new WaitFull(shooter, 3).deadlineWith(new Shoot(shooter)),
        new Goto(drive, new Pose2d(-Units.feetToMeters(feet), 0.0, new Rotation2d(0.0, 0.0)), true)
      ));
  }
}