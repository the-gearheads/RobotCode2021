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
import edu.wpi.first.wpiutil.net.PortForwarder;
import frc.robot.commands.angle.AngleCalibrate;
import frc.robot.commands.auton.AccuracyChallenge;
import frc.robot.commands.drive.Goto;
import frc.robot.commands.drive.Pathweaver;
import frc.robot.profile.AkhilDrive;
import frc.robot.profile.DebugDrive;
import frc.robot.profile.DebugOperate;
import frc.robot.profile.DriverProfile;
import frc.robot.profile.JuliaDrive;
import frc.robot.profile.OperatorProfile;
import frc.robot.profile.RobDrive;
import frc.robot.profile.SaadDrive;
import frc.robot.profile.StevenOperate;
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

  // Dashboard choosers
  private SendableChooser<Command> chooser = new SendableChooser<>();
  private SendableChooser<DriverProfile> driverChooser = new SendableChooser<>();
  private SendableChooser<OperatorProfile> operatorChooser = new SendableChooser<>();

  // Profiles
  public static DriverProfile driverProfile;
  public static OperatorProfile operatorProfile;

  public RobotContainer() {
    controller = new XboxController(Constants.CONTROLLER_PORT);
    joystick = new Joystick(Constants.JOYSTICK_PORT);
    streamdeck = new StreamDeck(0, 15);

    drive = new DriveSubsystem();
    shooter = new Shooter();
    angle = new ShooterAngle();
    intake = new Intake();
    elevator = new Elevator(intake);
    arms = new Arms();

    subsystems = new Subsystems(drive, shooter, angle, elevator, intake, arms);

    chooser.setDefaultOption("Accuracy Challenge", new AccuracyChallenge(drive, shooter, intake, elevator, angle));
    chooser.addOption("Barrel Race", new Pathweaver(drive, "barrel race"));
    chooser.addOption("Return to Origin", new Goto(drive, new Pose2d(0, 0, new Rotation2d(0)), false));
    chooser.addOption("Calibrate Shooter Angle", new AngleCalibrate(angle));
    SmartDashboard.putData("Select Auton", chooser);

    driverChooser.setDefaultOption("Debug", new DebugDrive());
    driverChooser.addOption("Akhil", new AkhilDrive());
    driverChooser.addOption("Julia", new JuliaDrive());
    driverChooser.addOption("Rob", new RobDrive());
    driverChooser.addOption("Saad", new SaadDrive());
    SmartDashboard.putData("Select Driver", driverChooser);

    operatorChooser.setDefaultOption("Debug", new DebugOperate());
    operatorChooser.addOption("Steven", new StevenOperate());
    SmartDashboard.putData("Select Operator", operatorChooser);

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

    driverProfile = driverChooser.getSelected();
    operatorProfile = operatorChooser.getSelected();

    driverProfile.createBinds(controller, joystick, streamdeck, subsystems);
    operatorProfile.createBinds(controller, joystick, streamdeck, subsystems);
  }

  public void disable() {
    NetworkTableInstance.getDefault().getTable("OpenSight").getEntry("led").setBoolean(false);
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

}
