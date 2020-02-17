/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.test.Spin;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.AngleCharacterize;
import frc.robot.util.StreamDeck;
import frc.robot.util.StreamDeckButton;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.Shooter;
// import frc.robot.util.JoystickTrigger;
// import frc.robot.commands.shooter.Elevator;
// import frc.robot.commands.shooter.Shot;

public class RobotContainer {
  private static XboxController controller;
  private static StreamDeck streamdeck;
  private static DriveSubsystem drive;
  // private static Shooter shooter;

  public RobotContainer() {
    drive = new DriveSubsystem();
    // shooter = new Shooter();

    controller = new XboxController(Constants.CONTROLLER_PORT);
    streamdeck = new StreamDeck(0, 15);
    configureButtonBindings();
  }

  // run on any mode init
  public void init() {
    streamdeck.reset();
  }

  public static XboxController getController() {
    return controller;
  }

  private void configureButtonBindings() {
    new JoystickButton(controller, XboxController.Button.kA.value).whenPressed(new AngleCharacterize(drive));
    new StreamDeckButton(streamdeck, 0, "arms up").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 1, "intake out").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 2).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 3).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 4, "shoot").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 5).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 6).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 7, "rotate").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 8).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 9, "unjam").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 10, "arms down").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 11, "intake").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 12).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 13).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 14, "aim").whenPressed(new Spin(drive).withTimeout(1));

    // JoystickTrigger lTrigger = new JoystickTrigger(controller,
    // XboxController.Axis.kLeftTrigger, 0.9);
    // lTrigger.whileHeld(new Elevator(shooter));
    // JoystickTrigger rTrigger = new JoystickTrigger(controller,
    // XboxController.Axis.kRightTrigger, 0.9);
    // rTrigger.whileHeld(new Shot(shooter));
    // SequentialCommandGroup group = new SequentialCommandGroup(new
    // Shot(shooter).withTimeout(1.5),
    // new ParallelCommandGroup(new Shot(shooter), new
    // Elevator(shooter)).withTimeout(2));
    // JoystickButton buttonA = new JoystickButton(controller,
    // XboxController.Button.kA.value);
    // buttonA.whenPressed(group);
  }

  public Command getAutonomousCommand() {
    final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        Constants.leftFF, drive.getKinematics(), 10);
    final TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_VELOCITY, Constants.MAX_ACCEL)
        .setKinematics(drive.getKinematics()).addConstraint(autoVoltageConstraint);
    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory, 
        () -> drive.getPose(),
        new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA), 
        Constants.leftFF, drive.getKinematics(),
        () -> drive.getWheelSpeeds(), 
        drive.controller.leftPid, 
        drive.controller.rightPid, 
        (left, right) -> drive.tankDriveVolts(left, right),
        drive
      );
    return ramseteCommand;
  }

}
