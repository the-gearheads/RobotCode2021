package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

public enum ACSet {
    GREEN(new AccuracySettings(new Pose2d(0, 0, new Rotation2d(0)), 5000, 40)),
    YELLOW(new AccuracySettings(new Pose2d(-Units.feetToMeters(5), 0, new Rotation2d(0)), 4250, 47.5)),
    BLUE(new AccuracySettings(new Pose2d(-Units.feetToMeters(10), 0, new Rotation2d(0)), 5250, 60)),
    RED(new AccuracySettings(new Pose2d(-Units.feetToMeters(15), 0, new Rotation2d(0)), 5750, 65.5));

    public final AccuracySettings value;

    ACSet(AccuracySettings value) {
        this.value = value;
    }
}