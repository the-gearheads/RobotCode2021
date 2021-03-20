/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpiutil.net.PortForwarder;
import frc.robot.commands.angle.AngleCalibrate;
import frc.robot.commands.angle.DriveAngle;
import frc.robot.commands.angle.SetAngle;
import frc.robot.commands.arms.WinchHold;
import frc.robot.commands.auton.AccuracyChallenge;
import frc.robot.commands.drive.Goto;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.util.StreamDeck;
import frc.robot.util.StreamDeckButton;
import frc.robot.util.Subsystems;

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
  private static Subsystems subsystems;

  // Dashboard
  private SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    drive = new DriveSubsystem(Constants.driverProfile);
    shooter = new Shooter();
    angle = new ShooterAngle();
    intake = new Intake();
    elevator = new Elevator(intake);
    arms = new Arms();

    subsystems = new Subsystems(drive, shooter, angle, elevator, intake, arms);

    controller = new XboxController(Constants.CONTROLLER_PORT);
    joystick = new Joystick(Constants.JOYSTICK_PORT);
    streamdeck = new StreamDeck(0, 15);
    setupStreamDeck();
    configureButtonBindings();

    chooser.addOption("Accuracy Challenge", new AccuracyChallenge(drive, shooter, intake, elevator, angle));
    chooser.addOption("Return to Origin", new Goto(drive, new Pose2d(0, 0, new Rotation2d(0)), false));
    chooser.addOption("Calibrate Shooter Angle", new AngleCalibrate(angle));

    SmartDashboard.putData("Auton Selector", chooser);

    PortForwarder.add(8000, "10.11.89.100", 80);
    PortForwarder.add(2200, "10.11.89.100", 22);
    PortForwarder.add(5800, "10.11.89.100", 5800);
    PortForwarder.add(554, "10.11.89.100", 554);
    CameraServer.getInstance().startAutomaticCapture(0);
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

    Constants.driverProfile.createBinds(controller, joystick, streamdeck, subsystems);
    Constants.operatorProfile.createBinds(controller, joystick, streamdeck, subsystems);

    // Supplier<Boolean> thirtySeconds = () ->
    // ((DriverStation.getInstance().getMatchTime() <= 30)
    // && DriverStation.getInstance().isOperatorControl());

    // new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.9)
    // .whileHeld(new SpeedModifier(drive, Constants.SLOW_MULTIPLIER, (1 / (double)
    // 2)));
    // new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.9)
    // .whileHeld(new SpeedModifier(drive, Constants.FAST_MULTIPLIER,
    // 1.5)).whenPressed(new SetSafety(drive, false))
    // .whenReleased(new SetSafety(drive, true));
    // new JoystickButton(controller, XboxController.Button.kA.value)
    // .whenPressed(new DriveToWall(drive, 1).withTimeout(0.6));
    // new JoystickButton(controller, XboxController.Button.kBumperRight.value)
    // .whenPressed(CommandScheduler.getInstance().cancelAll())
    // .whenPressed((new FullIntake(intake)).alongWith(new
    // Retract(intake))).whenPressed(this::init);
    // new JoystickButton(controller,
    // XboxController.Button.kBumperLeft.value).whenPressed(new Extend(intake));
    // new JoystickButton(controller,
    // XboxController.Button.kBumperRight.value).whenPressed(new Retract(intake));
    // new JoystickButton(controller, XboxController.Button.kB.value).whileHeld(new
    // FullIntake(intake)); new JoystickTrigger(controller, XboxControll
    // er.Axis.kLeftTrigger,
    // 0.1).whileHeld(new Elevate(elevator));
    // new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.1)
    // .whileHeld((new ShootAt(shooter).withTimeout(1)).andThen(
    // ShootAt(shooter))));

    // .whileHeld((new FullIntake(intake)).alongWith(new
    // Extend(intake)).alongWith(new Elevate(elevator)))
    // .whenReleased((new Retract(intake)
    // .alongWith((new Elevate(elevator).withTimeout(1)).alongWith(new Pft(intake,
    // false).withTimeout(1.5)))));

    // new JoystickButton(controller,
    // XboxController.Button.kA.value).whenPressed(new SetAngle(angle, 45));
    // new JoystickButton(controller,
    // XboxController.Button.kB.value).whenPressed(new SetAngle(angle, 20));
    // new JoystickButton(controller,
    // XboxController.Button.kX.value).whenPressed(new SetAngle(angle, 5));

    // new JoystickButton(controller,
    // XboxController.Button.kA.value).whenPressed(new Extend(intake));

    // new JoystickButton(controller, XboxController.Button.kA.value)
    // .whenPressed(new Goto(drive, new Pose2d(-Units.feetToMeters(5), 0.0, new
    // Rotation2d(0.0, 0.0)), true));
    // new JoystickButton(controller, XboxController.Button.kB.value)
    // .whenPressed(new Goto(drive, new Pose2d(-Units.feetToMeters(10), 0.0, new
    // Rotation2d(0.0, 0.0)), true));
    // new JoystickButton(controller, XboxController.Button.kY.value)
    // .whenPressed(new Goto(drive, new Pose2d(-Units.feetToMeters(15), 0.0, new
    // Rotation2d(0.0, 0.0)), true));

    //// THE THING AKHIL WANTED (cringe)
    // new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.1)
    // .whileHeld((new FullIntake(intake)).alongWith(new
    //// Elevate(elevator)).alongWith(new ShootAt(shooter)))
    // .whenPressed((new Extend(intake).alongWith(new
    //// NOP()).withTimeout(3).andThen(new Retract(intake))));

    // new JoystickButton(controller,
    // XboxController.Button.kY.value).whenPressed(new Extend(intake));
    // new JoystickButton(controller,
    // XboxController.Button.kX.value).whenPressed(new Retract(intake));
    // new JoystickButton(controller, XboxController.Button.kA.value)
    // .whileHeld(new FullIntake(intake).alongWith(new Elevate(elevator)));
    // new JoystickButton(controller, XboxController.Button.kB.value)
    // .whileHeld(new Shoot(shooter));

    new JoystickButton(joystick, 2).whileHeld(new DriveAngle(angle));
    new JoystickButton(joystick, 7).whenPressed(new SetAngle(angle, 45));
    new JoystickButton(joystick, 9).whenPressed(new SetAngle(angle, 28));
    new JoystickButton(joystick, 11).whenPressed(new SetAngle(angle, 0));
    new JoystickButton(joystick, 4).whileHeld(new WinchHold(arms, -1).withTimeout(10));
    new JoystickButton(joystick, 6).whileHeld(new WinchHold(arms, 1).withTimeout(8));

    // buttons[0].setIcon("arms up").addAutoStatus(thirtySeconds)
    // .whenPressed(new Winch(arms, Constants.WINCH_ROTATIONS, 1));
    // buttons[1].setIcon("arms up").setMode("hold").whileHeld(new WinchHold(arms,
    // 1));
    // buttons[2].setIcon("arms down").setMode("hold").whileHeld(new WinchHold(arms,
    // -1));
    // buttons[3].setIcon("aim").whenPressed(new TurnToAngle(drive));
    // buttons[4].setIcon("blue").setMode("hold").whileHeld((new
    // FullIntake(intake)).alongWith(new Extend(intake)))
    // .whenReleased((new Retract(intake)));
    // buttons[5].setIcon("yellow").setMode("hold").whileHeld(new
    // CloseShoot(shooter, angle, elevator));
    // buttons[6].setIcon("green");
    // buttons[9].setIcon("unjam").setMode("hold").whileHeld(new Unjam(angle,
    // elevator, shooter));
    // buttons[10].setIcon("blue").setMode("hold").whileHeld(new Pft(intake,
    // true).alongWith(new Extend(intake)))
    // .whenReleased(new Retract(intake));
    // buttons[11].setIcon("red");
    // buttons[12].setIcon("rotate");
    // buttons[14].setIcon("down").whenPressed(new SetAngle(angle, 0));
  }

  public static Intake getIntake() {
    return intake;
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public void routeTo(Pose2d pose) {

  }

}
