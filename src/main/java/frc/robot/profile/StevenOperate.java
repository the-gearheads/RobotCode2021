package frc.robot.profile;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.commands.angle.SetAngle;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.intake.Extend;
import frc.robot.commands.intake.FullIntake;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.shooter.ShootAt;
import frc.robot.util.StreamDeck;
import frc.robot.util.Subsystems;

public class StevenOperate extends OperatorProfile {

    public void createBinds(XboxController controller, Joystick joystick, StreamDeck streamdeck, Subsystems s) {
        setupStreamDeck(streamdeck);

        buttons[0].setMode("hold").setIcon("intake").setStatus(true)
                .whileHeld(new Extend(s.intake).alongWith(new FullIntake(s.intake)).alongWith(new Elevate(s.elevator)))
                .whenReleased(
                        new Retract(s.intake).alongWith((new Elevate(s.elevator).alongWith(new FullIntake(s.intake)).withTimeout(0.5))));
        buttons[4].setMode("hold").setIcon("shoot").setStatus(true)
                .whileHeld((new ShootAt(s.shooter).withTimeout(1)).andThen(new Elevate(s.elevator).alongWith(new ShootAt(s.shooter))));

        buttons[10].whenPressed(new SetAngle(s.angle, Constants.SHOOTER_ANGLE_MIN));
        buttons[11].whenPressed(new SetAngle(s.angle, 45));
        buttons[12].whenPressed(new SetAngle(s.angle, 47.5));
        buttons[13].whenPressed(new SetAngle(s.angle, 50));
        buttons[14].whenPressed(new SetAngle(s.angle, Constants.SHOOTER_ANGLE_MAX));
    }

}
