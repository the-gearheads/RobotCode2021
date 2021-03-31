// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.NOP;
import frc.robot.commands.angle.SetAngle;
import frc.robot.commands.drive.Goto;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.group.BlockedElevate;
import frc.robot.commands.intake.Extend;
import frc.robot.commands.intake.FullIntake;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.intake.WaitElevate;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.util.ACSet;
import frc.robot.util.AccuracySettings;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AccuracyChallenge extends ParallelDeadlineGroup {
  /** Creates a new AccuracyChallenge. */
  public AccuracyChallenge(DriveSubsystem drive, Shooter shooter, Intake intake, Elevator elevator, ShooterAngle angle) {
    super(new NOP(), new NOP());

    ArrayList<Command> commands = new ArrayList<Command>();

    AccuracySettings settings[] = new AccuracySettings[]{
      ACSet.GREEN.value,
      ACSet.YELLOW.value,
      ACSet.YELLOW.value,
      ACSet.BLUE.value,
      ACSet.BLUE.value,
      ACSet.RED.value,
    };

    // start by shooting 3 pre-loaded balls (rev up for .5 seconds, shoot for 2 seconds)
    commands.add(new Shoot(shooter, ACSet.GREEN.value.rpm).withTimeout(.5));
    commands.add((new Shoot(shooter, ACSet.GREEN.value.rpm).alongWith(new Elevate(elevator))).withTimeout(2));

    // iter 0: 2 green
    // iter 1: 2 yellow
    // iter 2: 2 yellow
    // iter 3: 2 blue
    // iter 4: 2 blue
    // iter 5: 2 red
    for (AccuracySettings setting : settings) {
      commands.add(new Goto(drive, new Pose2d(-Units.feetToMeters(20), 0, new Rotation2d(0)), true));
      commands.add(new WaitElevate(elevator, intake, 2));
      commands.add(new Goto(drive, setting.pose, false).alongWith(new SetAngle(angle, setting.angle)));
      commands.add(new Shoot(shooter, setting.rpm).withTimeout(.5));
      commands.add((new Shoot(shooter, setting.rpm).alongWith(new Elevate(elevator))).withTimeout(3.5));
    }

    commands.add(new Retract(intake));

    Command[] commandsArray = commands.toArray(new Command[0]);

    setDeadline(
      new SequentialCommandGroup(
        commandsArray
      ));

    addCommands(
      new SetAngle(angle, ACSet.GREEN.value.angle),
      new Extend(intake), 
      new FullIntake(intake)
    );
  }
}