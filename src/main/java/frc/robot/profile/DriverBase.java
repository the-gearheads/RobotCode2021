package frc.robot.profile;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.StreamDeck;
import frc.robot.util.Subsystems;
import frc.robot.util.Tuple;

public class DriverBase implements DriverProfile {

    public DriverSettings getSettings() {
        return new DriverSettings(4, 240, (1 / 3d), 1.5);
    }

    public void createBinds(XboxController controller, Joystick joystick, StreamDeck streamdeck, Subsystems s) {};

    public Tuple getArcadeAxis(XboxController controller) {
        double x = controller.getRawAxis(4);
        double y = controller.getRawAxis(1);
        return new Tuple(x, y);
    }

    public Command getDriveCommand(DriveSubsystem drive) {
        return new ArcadeDrive(drive);
    }
}
