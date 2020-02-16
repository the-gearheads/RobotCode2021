/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.Elevator;
import frc.robot.commands.shooter.Shot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.util.JoystickTrigger;

public class RobotContainer {
  public static XboxController controller;
  private static DriveSubsystem drive;
  // private static Intake intake;
  // private final SequentialCommandGroup ballGrab;
  private static Shooter shooter;

  public RobotContainer() {
    controller = new XboxController(Constants.CONTROLLER_PORT);
    drive = new DriveSubsystem();
    shooter = new Shooter();
    configureButtonBindings();
    // ballGrab = new SequentialCommandGroup(new Extend(intake), new Intake(intake));
  }

  private void configureButtonBindings() {
    // JoystickButton buttonLB = new JoystickButton(controller, XboxController.Button.kBumperLeft.value);
    // buttonLB.whenPressed(ballGrab);
    // JoystickButton buttonRB = new JoystickButton(controller, XboxController.Button.kBumperRight.value);
    // buttonRB.whenPressed(new Retract(intake));
    JoystickTrigger lTrigger = new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.9);
    lTrigger.whileHeld(new Elevator(shooter));
    JoystickTrigger rTrigger = new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.9);
    rTrigger.whileHeld(new Shot(shooter));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
