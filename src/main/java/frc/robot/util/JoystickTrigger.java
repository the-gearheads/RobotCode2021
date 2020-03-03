package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class JoystickTrigger extends Button {
    private XboxController controller;
    private XboxController.Axis axis;
    private double tolerance;

    public JoystickTrigger(XboxController controller, XboxController.Axis axis, double tolerance) {
        this.controller = controller;
        this.axis = axis;
        this.tolerance = tolerance;
    }

    public JoystickTrigger(BooleanSupplier isPressed) {
        super(isPressed);
    }

    @Override
    public boolean get() {
        return (controller.getRawAxis(axis.value) >= tolerance);
    }

    public Button and(JoystickTrigger trigger) {
        return new JoystickTrigger(() -> get() && trigger.get());
    }

    public JoystickTrigger or(JoystickTrigger trigger) {
        return new JoystickTrigger(() -> get() || trigger.get());
    }

    public JoystickTrigger negate() {
        return new JoystickTrigger(() -> !get());
    }
}