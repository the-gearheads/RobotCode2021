package frc.robot.util;

public class Deadband {
    public static double get(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        }
        return value;
    }

    public static double getSmart(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        }
        if (value < 0) {
            return (value + deadband) / (1 - deadband);
        }
        else {
            return (value - deadband) / (1 - deadband);
        }
    }
}