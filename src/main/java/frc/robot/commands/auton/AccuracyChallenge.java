// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.ArrayList;
import java.util.Collection;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.Intake;
import frc.robot.commands.NOP;
import frc.robot.commands.angle.SetAngle;
import frc.robot.commands.drive.Goto;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.intake.Extend;
import frc.robot.commands.intake.FullIntake;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.intake.WaitFull;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAll;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AccuracyChallenge extends ParallelDeadlineGroup {
  /** Creates a new AccuracyChallenge. */
  public AccuracyChallenge(DriveSubsystem drive, Shooter shooter, Intake intake, Elevator elevator, ShooterAngle shooterAngle) {
    super(new NOP(), new NOP());

    ArrayList<Command> commands = new ArrayList<Command>();

    double angles[] = {47.5, 50.0, 51.0, 52.5};
    double rpms[] = {5550, 5650, 6050, 6700};

    for (int i = 1; i < 5; i++) {
      commands.add((new WaitFull(shooter, 3).andThen(new NOP().withTimeout(0.5))).deadlineWith(new Shoot(shooter, rpms[i - 1])));
      if (i != 4) { // don't go back on last iter
        commands.add(new Goto(drive, new Pose2d(-Units.feetToMeters(5 * i), 0, new Rotation2d(0)), true).alongWith(new SetAngle(shooterAngle, angles[i - 1])));
      }
    }

    commands.add(new Retract(intake));

    Command[] commandsArray = commands.toArray(new Command[0]);

    setDeadline(
      new SequentialCommandGroup(
        commandsArray
      ));

    addCommands(
      new SetAngle(shooterAngle, angles[0]),
      new Elevate(elevator), 
      new Extend(intake), 
      new FullIntake(intake)
    );
  }
}