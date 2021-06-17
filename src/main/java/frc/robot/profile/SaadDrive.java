package frc.robot.profile;

import java.util.Optional;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.SpeedModifier;
import frc.robot.util.StreamDeck;
import frc.robot.util.Subsystems;

public class SaadDrive extends DriverBase {
    public DriverSettings getSettings() {
        return new DriverSettings(1, 120, (1 / (double) 3), 1.5);
    }

    public void createBinds(XboxController controller, Joystick joystick, StreamDeck streamdeck, Subsystems s) {
        new JoystickButton(controller, XboxController.Button.kX.value)
                .whileHeld(new SpeedModifier(s.drive, Optional.empty(), Optional.empty(), Optional.of(0.0)));
    };
}
