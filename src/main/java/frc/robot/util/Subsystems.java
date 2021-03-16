package frc.robot.util;

import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

public class Subsystems {
    public final DriveSubsystem drive;
    public final Shooter shooter;
    public final ShooterAngle angle;
    public final Elevator elevator;
    public final Intake intake;
    public final Arms arms;

    public Subsystems(DriveSubsystem drive, Shooter shooter, ShooterAngle angle, Elevator elevator, Intake intake, Arms arms) {
        this.drive = drive;
        this.shooter = shooter;
        this.angle = angle;
        this.elevator = elevator;
        this.intake = intake;
        this.arms = arms;
    }

}
