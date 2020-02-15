/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ballIntake.Extend;
import frc.robot.commands.ballIntake.Intake;
import frc.robot.commands.ballIntake.Retract;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extender;

public class RobotContainer {
  public static XboxController controller;
  private static DriveSubsystem drive;
  private static Extender extender;
  private final SequentialCommandGroup ballGrab;

  public RobotContainer() {
    controller = new XboxController(Constants.CONTROLLER_PORT);
    drive = new DriveSubsystem();
    configureButtonBindings();
    ballGrab = new SequentialCommandGroup(new Extend(extender), new Intake(extender));
  }

  private void configureButtonBindings() {
    JoystickButton buttonLB = new JoystickButton(controller, XboxController.Button.kBumperLeft.value);
    buttonLB.whenPressed(ballGrab);
    JoystickButton buttonRB = new JoystickButton(controller, XboxController.Button.kBumperRight.value);
    buttonRB.whenPressed(new Retract(extender));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

