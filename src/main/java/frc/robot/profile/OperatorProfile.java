package frc.robot.profile;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.StreamDeck;
import frc.robot.util.StreamDeckButton;
import frc.robot.util.Subsystems;

public abstract class OperatorProfile {

    protected StreamDeckButton[] buttons;

    // Must be run in order to populate buttons
    protected void setupStreamDeck(StreamDeck streamdeck) {
        int count = streamdeck.getButtons();
        buttons = new StreamDeckButton[count];
        for (int i = 0; i <= (count - 1); i++) {
            buttons[i] = new StreamDeckButton(streamdeck, i);
        }
    }

    public abstract void createBinds(XboxController controller, Joystick joystick, StreamDeck streamdeck,
            Subsystems subsystems);

}
