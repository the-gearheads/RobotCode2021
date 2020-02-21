package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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
    private final DifferentialDriveKinematics m_kinematics;

    private double m_prevTime;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;

    public Ramsete(Trajectory trajectory, RamseteController controller, DriveSubsystem drive) {
        m_trajectory = trajectory;
        m_controller = controller;
        m_drive = drive;
        m_kinematics = drive.getKinematics();

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        ChassisSpeeds chs = (m_controller.calculate(m_drive.getPose(), m_trajectory.sample(curTime)));
        DifferentialDriveWheelSpeeds targetWheelSpeeds = m_kinematics.toWheelSpeeds(chs);
        m_drive.controller.tankDrive(targetWheelSpeeds);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
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
