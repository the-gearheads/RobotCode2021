/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Collections;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.commands.angle.DriveAngle;
import frc.robot.commands.angle.SetAngle;
import frc.robot.commands.arms.Winch;
import frc.robot.commands.arms.WinchHold;
import frc.robot.commands.drive.SpeedModifier;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.group.BlockedElevate;
import frc.robot.commands.group.Unjam;
import frc.robot.commands.intake.Extend;
import frc.robot.commands.intake.FullIntake;
import frc.robot.commands.intake.Pft;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAll;
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
  public static Joystick joystick;
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
    joystick = new Joystick(Constants.JOYSTICK_PORT);
    streamdeck = new StreamDeck(0, 15);
    setupStreamDeck();
    configureButtonBindings();
  }

  // run on any mode init
  public void init() {
    streamdeck.reset();
    NetworkTableInstance.getDefault().getTable("OpenSight").getEntry("led").setBoolean(true);
  }

  public void disable() {
    NetworkTableInstance.getDefault().getTable("OpenSight").getEntry("led").setBoolean(false);
  }

  public static StreamDeckButton getButton(int index) {
    return buttons[index];
  }

  private void setupStreamDeck() {
    int count = streamdeck.getButtons();
    buttons = new StreamDeckButton[count];
    for (int i = 0; i <= (count - 1); i++) {
      buttons[i] = new StreamDeckButton(streamdeck, i);
    }
  }

  private void configureButtonBindings() {
    Supplier<Boolean> thirtySeconds = () -> ((DriverStation.getInstance().getMatchTime() <= 30) && DriverStation.getInstance().isOperatorControl());

    new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.9)
        .whileHeld(new SpeedModifier(drive, Constants.SLOW_MULTIPLIER));
    new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.9)
        .whileHeld(new SpeedModifier(drive, Constants.FAST_MULTIPLIER));

    new JoystickButton(joystick, 1).whileHeld((new Shoot(shooter).withTimeout(1)).andThen((new ShootAll(shooter)).deadlineWith(new Elevate(elevator))));
    new JoystickButton(joystick, 2).whileHeld(new DriveAngle(angle));
    new JoystickButton(joystick, 7).whenPressed(new SetAngle(angle, 45));
    new JoystickButton(joystick, 9).whenPressed(new SetAngle(angle, 20));
    new JoystickButton(joystick, 11).whenPressed(new SetAngle(angle, 0));

    buttons[0].setIcon("arms up").addAutoStatus(thirtySeconds).whenPressed(new Winch(arms, 40, 1));
    buttons[1].setIcon("arms up").addAutoStatus(thirtySeconds).setMode("hold").whileHeld(new WinchHold(arms, 1));
    buttons[2].setIcon("arms down").setMode("hold").whileHeld(new WinchHold(arms, -1));
    buttons[3].setIcon("aim").whenPressed(new TurnToAngle(drive));
    buttons[4].setIcon("intake").setMode("hold").whileHeld((new FullIntake(intake)).alongWith(new Extend(intake))).whenReleased((new Retract(intake)).alongWith((new Pft(intake)).withTimeout(5)));
    buttons[5].setIcon("yellow");
    buttons[6].setIcon("green");
    buttons[9].setIcon("unjam").setMode("hold").whileHeld(new Unjam(angle, elevator, shooter));
    buttons[10].setIcon("blue");
    buttons[11].setIcon("red");
    buttons[12].setIcon("rotate");
    buttons[14].setIcon("down").whenPressed(new SetAngle(angle, 0));
  }

  public static Shooter getShooter() {
    return shooter;
  }

  public static Intake getIntake() {
    return intake;
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
