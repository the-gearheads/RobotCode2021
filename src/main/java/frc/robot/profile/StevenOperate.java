package frc.robot.profile;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.angle.AngleCalibrate;
import frc.robot.commands.angle.HoldAngle;
import frc.robot.commands.angle.IncrementAngle;
import frc.robot.commands.angle.SetAngle;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.intake.Extend;
import frc.robot.commands.intake.FullIntake;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.intake.SetExtended;
import frc.robot.commands.intake.WaitElevate;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAt;
import frc.robot.util.StreamDeck;
import frc.robot.util.Subsystems;

public class StevenOperate extends OperatorProfile {

    public void createBinds(XboxController controller, Joystick joystick, StreamDeck streamdeck, Subsystems s) {
        setupStreamDeck(streamdeck);
        buttons[0].setMode("hold").setIcon("near").whileHeld(shootAt(s, 5000, 41));
        buttons[1].setMode("hold").setIcon("medium").whileHeld(shootAt(s, 5000, 54));
        buttons[2].setMode("hold").setIcon("far").whileHeld(shootAt(s, 5300, 58));
        buttons[5].setMode("hold").setIcon("intake").setStatus(false)
                .whileHeld(new Extend(s.intake).alongWith(new FullIntake(s.intake)))
                .whenReleased(new Retract(s.intake).alongWith((new FullIntake(s.intake).withTimeout(0.5))));
        buttons[6].setMode("hold").setIcon("intake").setStatus(true)
                .whileHeld((new Extend(s.intake).deadlineWith(new FullIntake(s.intake)))
                        .andThen(new WaitElevate(s.elevator, s.intake, 3).deadlineWith(new FullIntake(s.intake))))
                .whenReleased(new Retract(s.intake));
        buttons[7].setIcon("aim").setStatus(true).whenPressed(new AngleCalibrate(s.angle));
        buttons[8].setMode("hold").setIcon("blue").setStatus(true).whileHeld(new Elevate(s.elevator));

    }

    public Command shootAt(Subsystems s, double rpm) {
        return (new Shoot(s.shooter, rpm).withTimeout(1))
                .andThen(new Elevate(s.elevator).alongWith(new Shoot(s.shooter, rpm)));
    }

    public Command shootAt(Subsystems s, double rpm, double angle) {
        return new Shoot(s.shooter, rpm).alongWith(new SetAngle(s.angle, angle));
    }

}
