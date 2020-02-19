/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.test.Spin;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.AngleCharacterize;
import frc.robot.util.StreamDeck;
import frc.robot.util.StreamDeckButton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.util.JoystickTrigger;
import frc.robot.commands.shooter.Elevator;
import frc.robot.commands.shooter.Shot;

public class RobotContainer {
  public static XboxController controller;
  private static StreamDeck streamdeck;
  private static DriveSubsystem drive;
  private static Shooter shooter;

  public RobotContainer() {
    drive = new DriveSubsystem();
    shooter = new Shooter();

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

    SequentialCommandGroup shootGroup = new SequentialCommandGroup(new Shot(shooter).withTimeout(1.5),
        new ParallelCommandGroup(new Shot(shooter), new Elevator(shooter)).withTimeout(2));
    new JoystickButton(controller, XboxController.Button.kA.value).whenPressed(new AngleCharacterize(drive));
    new StreamDeckButton(streamdeck, 0, "arms up").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 1, "intake out").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 2, "red").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 3, "elevator plus").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 4, "shoot");
    new StreamDeckButton(streamdeck, 5, "color wheel").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 6, "green").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 7, "rotate").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 8, "yellow").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 9, "unjam").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 10, "arms down").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 11, "intake").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 12, "blue").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 13, "elevator minus").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 14, "aim").whenPressed(new Spin(drive).withTimeout(1));

    JoystickTrigger lTrigger = new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.9);
    lTrigger.whileHeld(new Elevator(shooter));
    JoystickTrigger rTrigger = new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.9);
    rTrigger.whileHeld(new Shot(shooter));
    JoystickButton buttonA = new JoystickButton(controller, XboxController.Button.kA.value);
    buttonA.whenPressed(shootGroup);
  }

  public Command getAutonomousCommand() {
    return null;
  }

}
