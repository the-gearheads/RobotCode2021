/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arms.ExtendArms;
import frc.robot.commands.arms.RetractArms;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.Elevator;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShooterAngle;
import frc.robot.commands.spinner.SpinColor;
import frc.robot.commands.spinner.SpinRotations;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;
import frc.robot.util.AngleCharacterize;
import frc.robot.util.JoystickTrigger;
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
    SequentialCommandGroup shootGroup = new SequentialCommandGroup(new Shoot(shooter).withTimeout(1.5),
        new ParallelCommandGroup(new Shoot(shooter), new Elevator(shooter)).withTimeout(2));

    // Set up joystick binds
    new JoystickButton(controller, XboxController.Button.kA.value).whenPressed(new AngleCharacterize(drive));
    new JoystickButton(controller, XboxController.Button.kX.value).whenPressed(new ShooterAngle(shooter));
    JoystickTrigger lTrigger = new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.9);
    lTrigger.whileHeld(new Elevator(shooter));
    JoystickTrigger rTrigger = new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.9);
    rTrigger.whileHeld(new Shoot(shooter));

    // Set up StreamDeck buttons 
    new StreamDeckButton(streamdeck, 0, "arms up").whenPressed(new ExtendArms(arms).withTimeout(5)); // TODO: Set timing
    new StreamDeckButton(streamdeck, 1, "intake out").whenPressed(new RunIntake(intake, -0.8)); // TODO: Bind
    new StreamDeckButton(streamdeck, 2, "red").whenPressed(new SpinColor(spinner, "Red"));
    new StreamDeckButton(streamdeck, 3, "elevator plus"); // TODO: Bind
    new StreamDeckButton(streamdeck, 4, "shoot").whenPressed(shootGroup);
    new StreamDeckButton(streamdeck, 5, "color wheel"); // TODO: Bind
    new StreamDeckButton(streamdeck, 6, "green").whenPressed(new SpinColor(spinner, "Green"));
    new StreamDeckButton(streamdeck, 7, "rotate").whenPressed(new SpinRotations(spinner, 4));
    new StreamDeckButton(streamdeck, 8, "yellow").whenPressed(new SpinColor(spinner, "Yellow"));
    new StreamDeckButton(streamdeck, 9, "unjam"); // TODO: Bind
    new StreamDeckButton(streamdeck, 10, "arms down").whenPressed(new RetractArms(arms).withTimeout(5)); // TODO: Set timing
    new StreamDeckButton(streamdeck, 11, "intake").whenPressed(new RunIntake(intake, 0.8));
    new StreamDeckButton(streamdeck, 12, "blue").whenPressed(new SpinColor(spinner, "Blue"));
    new StreamDeckButton(streamdeck, 13, "elevator minus"); // TODO: Bind
    new StreamDeckButton(streamdeck, 14, "aim").whenPressed(this::turnToAngle);

  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void turnToAngle() {
    (new TurnToAngle(drive, cameraAngle.getDouble(0), 1, true)).schedule();
  }

}
