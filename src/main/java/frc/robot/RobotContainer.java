/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.test.GetAngleData;
import frc.robot.commands.test.RevAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.AngleCharacterize;

public class RobotContainer {
  public static XboxController controller;
  private static DriveSubsystem drive;

  public RobotContainer() {
    controller = new XboxController(Constants.CONTROLLER_PORT);
    drive = new DriveSubsystem();
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(controller, XboxController.Button.kA.value).whenPressed(new AngleCharacterize(drive));
  }

  public Command getAutonomousCommand() {
    return null;
  }

}
