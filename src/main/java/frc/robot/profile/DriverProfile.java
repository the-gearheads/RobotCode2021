package frc.robot.profile;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.StreamDeck;
import frc.robot.util.Subsystems;
import frc.robot.util.Tuple;

public interface DriverProfile {
    
    public DriverSettings getSettings();

    public void createBinds(XboxController controller, Joystick joystick, StreamDeck streamdeck, Subsystems subsystems);

    public Tuple getArcadeAxis(XboxController controller);

    public Command getDriveCommand(DriveSubsystem drive);

}
