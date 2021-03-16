package frc.robot.profile;

public class DriverSettings {

    // Meters per second
    public final double THROTTLE_SPEED;
    // Degrees per second
    public final double ROT_SPEED;

    public final double SLOW_MULTIPLIER;
    public final double FAST_MULTIPLIER;

    public DriverSettings(double throttleSpeed, double rotSpeed, double slowMultiplier, double fastMultiplier) {
        THROTTLE_SPEED = throttleSpeed;
        ROT_SPEED = rotSpeed;
        SLOW_MULTIPLIER = slowMultiplier;
        FAST_MULTIPLIER = fastMultiplier;
    }

}
