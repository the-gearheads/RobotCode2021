package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class AccuracySettings {
    public Pose2d pose;
    public int rpm;
    public double angle;

    public AccuracySettings(Pose2d pose, int rpm, double angle) {
        this.pose = pose;
        this.rpm = rpm;
        this.angle = angle;
    }
}
