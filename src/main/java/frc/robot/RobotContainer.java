/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Collections;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.group.BlockedElevate;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.util.JoystickTrigger;
import frc.robot.util.Ramsete;
import frc.robot.util.StreamDeck;
import frc.robot.util.StreamDeckButton;

public class RobotContainer {
  // OI
  public static XboxController controller;
  private static StreamDeck streamdeck;
  private static StreamDeckButton[] buttons;

  // Subsystems
  private static DriveSubsystem drive;
  private static Shooter shooter;
  private static Elevator elevator;
  private static ShooterAngle angle;
  private static Intake intake;
  private static Arms arms;

  public RobotContainer() {
    drive = new DriveSubsystem();
    shooter = new Shooter();
    angle = new ShooterAngle();
    intake = new Intake();
    elevator = new Elevator();
    arms = new Arms();

    controller = new XboxController(Constants.CONTROLLER_PORT);
    streamdeck = new StreamDeck(0, 15);
    setupStreamDeck();
    configureButtonBindings();
  }

  // run on any mode init
  public void init() {
    streamdeck.reset();
  }

  public static StreamDeckButton getButton(int index) {
    return buttons[index];
  }

  private void setupStreamDeck() {
    int count = streamdeck.getButtons();
    buttons = new StreamDeckButton[count];
    for (int i = 0; i <= count; i++) {
      buttons[i] = new StreamDeckButton(streamdeck, i);
    }
  }

  private void configureButtonBindings() {
    // Set up joystick binds
    new JoystickButton(controller, XboxController.Button.kX.value).whenPressed(new TurnToAngle(drive));
    new JoystickButton(controller, XboxController.Button.kY.value).whenPressed(this::routeToOrigin);
    JoystickTrigger rTrigger = new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.9);
    rTrigger.whileHeld(
        (new Shoot(shooter).withTimeout(1)).andThen((new Shoot(shooter)).deadlineWith(new Elevate(elevator))));
    JoystickTrigger lTrigger = new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.9);
    lTrigger.whileHeld((new BlockedElevate(elevator, shooter).alongWith(new RunIntake(intake))));
  }

  public Command getAutonomousCommand() {
    final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        Constants.leftFF, drive.getKinematics(), 10);
    final TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_VELOCITY, Constants.MAX_ACCEL)
        .setKinematics(drive.getKinematics()).addConstraint(autoVoltageConstraint);
    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()),
        Collections.emptyList(), new Pose2d(1, 0, new Rotation2d()), config);

    Trajectory transformed = trajectory.transformBy(drive.getPose().minus(new Pose2d()));
    System.out.println(transformed.getTotalTimeSeconds());
    Ramsete ramseteCommand = new Ramsete(transformed,
        new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA), drive);
    ramseteCommand.schedule();
    return ramseteCommand;
  }

  public void routeToOrigin() {
    final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        Constants.leftFF, drive.kinematics, 2);
    final TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_VELOCITY, Constants.MAX_ACCEL)
        .setKinematics(drive.kinematics).addConstraint(autoVoltageConstraint);

    try {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(drive.getPose(), Collections.emptyList(),
          new Pose2d(0, 0, new Rotation2d(0)), config);
      Ramsete ramseteCommand = new Ramsete(trajectory,
          new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA), drive);
      ramseteCommand.schedule();
    } catch (MalformedSplineException e) {
      DriverStation.reportError("Failed to generate routeToOrigin trajectory", e.getStackTrace());
    }

  }

}
