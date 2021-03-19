package frc.robot.profile;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.drive.SpeedModifier;
import frc.robot.util.JoystickTrigger;
import frc.robot.util.StreamDeck;
import frc.robot.util.Subsystems;

public class JuliaDrive extends DriverBase {
    public void createBinds(XboxController controller, Joystick joystick, StreamDeck streamdeck, Subsystems s) {
        // new JoystickTrigger(controller, XboxController.Axis.kRightTrigger, 0.1)
        //     .whileHeld(new SpeedModifier(s.drive, s.drive.profile.getSettings().FAST_MULTIPLIER, s.drive.profile.getSettings().FAST_MULTIPLIER, 1));
        
    }

}
