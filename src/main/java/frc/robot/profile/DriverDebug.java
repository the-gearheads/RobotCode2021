package frc.robot.profile;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auton.AccuracyChallenge;
import frc.robot.commands.drive.Goto;
import frc.robot.commands.drive.Pathweaver;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.intake.Extend;
import frc.robot.commands.intake.FullIntake;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.intake.SetExtended;
import frc.robot.commands.shooter.ShootAt;
import frc.robot.util.JoystickTrigger;
import frc.robot.util.StreamDeck;
import frc.robot.util.Subsystems;

public class DriverDebug extends DriverBase {

    public void createBinds(XboxController controller, Joystick joystick, StreamDeck streamdeck, Subsystems s) {
        new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.1)
                .whileHeld((new ShootAt(s.shooter).withTimeout(1))
                        .andThen((new ShootAt(s.shooter)).deadlineWith(new Elevate(s.elevator))));

        new JoystickButton(controller, XboxController.Button.kB.value).whileHeld(new FullIntake(s.intake));
        new JoystickButton(controller, XboxController.Button.kY.value).whenPressed(new Pathweaver(s.drive, "autonav"));

        // GOTO TESTS (accuracy challenge)

        new JoystickButton(controller, XboxController.Button.kStart.value).whenPressed(
                new Goto(s.drive, new Pose2d(-Units.feetToMeters(0), 0.0, new Rotation2d(0.0, 0.0)), false));

        new JoystickButton(controller, XboxController.Button.kBack.value).whenPressed(new SetExtended(s.intake));
        new JoystickButton(controller, XboxController.Button.kX.value)
                .whenPressed(new AccuracyChallenge(s.drive, s.shooter, s.intake, s.elevator, s.angle));

        // REAL AUTOSHOOT
        new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.1)
                .whileHeld(new Extend(s.intake).alongWith(new FullIntake(s.intake)).alongWith(new Elevate(s.elevator))
                        .alongWith(new ShootAt(s.shooter)))
                .whenReleased(new Retract(s.intake).alongWith(
                        (new Elevate(s.elevator).alongWith(new FullIntake(s.intake)).alongWith(new ShootAt(s.shooter)))
                                .withTimeout(2)));

        new JoystickButton(controller, XboxController.Button.kA.value)
                .whileHeld(new FullIntake(s.intake).alongWith(new Elevate(s.elevator)));
    }
}
