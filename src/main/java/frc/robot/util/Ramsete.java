package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/*
It's like RamseteCommand, but without the bloaty nonsense
*/
public class Ramsete extends CommandBase {
    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final RamseteController m_controller;
    private final DriveSubsystem m_drive;

    public Ramsete(Trajectory trajectory, RamseteController controller, DriveSubsystem drive) {
        m_trajectory = trajectory;
        m_controller = controller;
        m_drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        var chs = (m_controller.calculate(m_drive.getPose(), m_trajectory.sample(curTime)));
        var targetWheelSpeeds = m_drive.getKinematics().toWheelSpeeds(chs);
        m_drive.controller.tankDrive(targetWheelSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }
}
