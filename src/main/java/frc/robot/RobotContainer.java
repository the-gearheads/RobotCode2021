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

public class RobotContainer {
  public static XboxController controller;
  private StreamDeck streamdeck;
  private static DriveSubsystem drive;

  public RobotContainer() {
    controller = new XboxController(Constants.CONTROLLER_PORT);
    drive = new DriveSubsystem();
    streamdeck = new StreamDeck(0, 15);
    configureButtonBindings();
  }

  // run on any mode init
  public void init() {
    streamdeck.reset();
  }

  private void configureButtonBindings() {
    new JoystickButton(controller, XboxController.Button.kA.value).whenPressed(new AngleCharacterize(drive));
    new StreamDeckButton(streamdeck, 0).setIcon("arms up").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 1).setIcon("intake out").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 2).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 3).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 4).setIcon("shoot").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 5).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 6).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 7).setIcon("rotate").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 8).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 9).setIcon("unjam").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 10).setIcon("arms down").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 11).setIcon("intake").whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 12).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 13).whenPressed(new Spin(drive).withTimeout(1));
    new StreamDeckButton(streamdeck, 14).setIcon("aim").whenPressed(new Spin(drive).withTimeout(1));
   
  }

  public Command getAutonomousCommand() {

    return null;
  }

}
