package frc.robot.profile;

import java.util.Optional;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.SpeedModifier;
import frc.robot.util.JoystickTrigger;
import frc.robot.util.StreamDeck;
import frc.robot.util.Subsystems;
import frc.robot.util.Tuple;

public class AkhilDrive extends DriverBase {

    public Tuple getArcadeAxis(XboxController controller) {
        double x = controller.getRawAxis(4);
        double y = controller.getRawAxis(1);
        return new Tuple(x, y);
    }

    public void createBinds(XboxController controller, Joystick joystick, StreamDeck streamdeck, Subsystems s) {
        new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.1)
            .whileHeld(new SpeedModifier(s.drive, Optional.of(0.0), Optional.empty(), Optional.empty()));

        new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.1)
            .whileHeld(new SpeedModifier(s.drive, Optional.empty(), Optional.of(0.0), Optional.empty()));

        new JoystickButton(controller, XboxController.Button.kB.value)
            .whileHeld(new SpeedModifier(s.drive, Optional.of(0.0), Optional.of(0.0), Optional.empty()));

        new JoystickButton(controller, XboxController.Button.kBumperRight.value)
            .whileHeld(new SpeedModifier(s.drive, Optional.of(s.drive.profile.getSettings().FAST_MULTIPLIER), Optional.of(s.drive.profile.getSettings().FAST_MULTIPLIER), Optional.empty()));
       
    }
}
