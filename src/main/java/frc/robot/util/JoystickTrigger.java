package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class JoystickTrigger extends Button {
    private final XboxController controller;
    private final XboxController.Axis axis;
    private final double tolerance;

    public JoystickTrigger(XboxController controller, XboxController.Axis axis, double tolerance) {
        this.controller = controller;
        this.axis = axis;
        this.tolerance = tolerance;
    }

    @Override
    public boolean get() {
        return (controller.getRawAxis(axis.value) >= tolerance);
    }

}