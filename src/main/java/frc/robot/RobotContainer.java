/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Collections;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.spline.SplineParameterizer.MalformedSplineException;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arms.ExtendArms;
import frc.robot.commands.arms.RetractArms;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.intake.Extend;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.Elevate;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAndElevate;
import frc.robot.commands.shooter.ShooterAngle;
import frc.robot.commands.spinner.SpinColor;
import frc.robot.commands.spinner.SpinRotations;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.util.JoystickTrigger;
import frc.robot.util.Ramsete;
import frc.robot.util.StreamDeck;
import frc.robot.util.StreamDeckButton;

public class RobotContainer {
  // OI
  public static XboxController controller;
  private static StreamDeck streamdeck;

  // Subsystems
  private static DriveSubsystem drive;
  private static Shooter shooter;
  private static Spinner spinner;
  private static Intake intake;
  private static Arms arms;

  // Misc
  private final NetworkTableEntry cameraAngle;

  public RobotContainer() {
    drive = new DriveSubsystem();
    shooter = new Shooter();
    spinner = new Spinner();
    intake = new Intake();
    arms = new Arms();

    controller = new XboxController(Constants.CONTROLLER_PORT);
    streamdeck = new StreamDeck(0, 15);
    configureButtonBindings();

    cameraAngle = NetworkTableInstance.getDefault().getTable("OpenSight").getEntry("camera");
    cameraAngle.setNumber(0);
    SmartDashboard.putNumber("shooterAngle", shooter.getAnglePosition());
  }

  // run on any mode init
  public void init() {
    streamdeck.reset();
  }

  public void createGroups() {
  }

  private void configureButtonBindings() {
    // Set up command groups
    // SequentialCommandGroup shootGroup = (new Shoot(shooter).withTimeout(1))
        // .andThen(new ShootAndElevate(shooter).withTimeout(2));

    // Set up joystick binds
    // new JoystickButton(controller, XboxController.Button.kA.value).whenPressed(shootGroup);
    new JoystickButton(controller, XboxController.Button.kA.value).whenPressed(new Extend(intake));
    new JoystickButton(controller, XboxController.Button.kB.value).whenPressed(new Retract(intake));
    new JoystickButton(controller, XboxController.Button.kX.value).whenPressed(new TurnToAngle(drive));
    new JoystickButton(controller, XboxController.Button.kY.value).whenPressed(this::routeToOrigin);
    JoystickTrigger lTrigger = new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.9);
    lTrigger.whileHeld(new RunIntake(intake, 0.4));
    JoystickTrigger rTrigger = new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.9);
    // rTrigger.whileHeld(new ShootAndElevate(shooter));

    // Set up StreamDeck buttons
    new StreamDeckButton(streamdeck, 0, "arms up").whenPressed(new ExtendArms(arms).withTimeout(5)); // TODO: Set timing
    new StreamDeckButton(streamdeck, 1, "intake out").whenPressed(new RunIntake(intake, -0.8)); // TODO: Bind
    new StreamDeckButton(streamdeck, 2, "red").whenPressed(new SpinColor(spinner, "Red"));
    new StreamDeckButton(streamdeck, 3, "elevator plus"); // TODO: Bind
    // new StreamDeckButton(streamdeck, 4, "shoot").whenPressed(shootGroup);
    new StreamDeckButton(streamdeck, 5, "color wheel"); // TODO: Bind
    new StreamDeckButton(streamdeck, 6, "green").whenPressed(new SpinColor(spinner, "Green"));
    new StreamDeckButton(streamdeck, 7, "rotate").whenPressed(new SpinRotations(spinner, 4));
    new StreamDeckButton(streamdeck, 8, "yellow").whenPressed(new SpinColor(spinner, "Yellow"));
    new StreamDeckButton(streamdeck, 9, "unjam"); // TODO: Bind
    new StreamDeckButton(streamdeck, 10, "arms down").whenPressed(new RetractArms(arms).withTimeout(5)); // TODO: timing
    new StreamDeckButton(streamdeck, 11, "intake").whenPressed(new RunIntake(intake, 0.8));
    new StreamDeckButton(streamdeck, 12, "blue").whenPressed(new SpinColor(spinner, "Blue"));
    new StreamDeckButton(streamdeck, 13, "elevator minus"); // TODO: Bind
    // new StreamDeckButton(streamdeck, 14, "aim").whenPressed(new TurnToAngle(drive));

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
