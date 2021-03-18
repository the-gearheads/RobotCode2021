package frc.robot.profile;

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
            .whileHeld(new SpeedModifier(s.drive, 0, 1, 0));

        new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.1)
            .whileHeld(new SpeedModifier(s.drive, 1, 0, 0));

        new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.1)
            .and(new JoystickTrigger(controller, XboxController.Axis.kLeftTrigger, 0.1)
                .whileHeld(new SpeedModifier(s.drive, 0, 0, 0)));
        
        new JoystickButton(controller, XboxController.Button.kB.value)
            .whileHeld(new SpeedModifier(s.drive, 0, 0, 0));

        new JoystickButton(controller, XboxController.Button.kBumperRight.value)
            .whileHeld(new SpeedModifier(s.drive, s.drive.profile.getSettings().FAST_MULTIPLIER, s.drive.profile.getSettings().FAST_MULTIPLIER, 0));
       
    }
}
