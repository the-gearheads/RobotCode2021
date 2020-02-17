package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.test.GetAngleData;
import frc.robot.commands.test.RevAngle;
import frc.robot.subsystems.DriveSubsystem;

public class AngleCharacterize implements Runnable {
    private double angle = 20;
    private CommandScheduler scheduler;
    private DriveSubsystem drive;
    
    public AngleCharacterize(DriveSubsystem drive) {
        this.drive = drive;
    }

    public void run() {
        SmartDashboard.putBoolean("Test/Done", false);
        scheduler = CommandScheduler.getInstance();
        scheduler.schedule(createGroup());
        scheduler.onCommandFinish((command) -> handleFinish(command));
    }

    public SequentialCommandGroup createGroup() {
        angle += 5;
        SmartDashboard.putNumber("Test/Target", angle);
        return new SequentialCommandGroup(new RevAngle(drive, angle).withTimeout(1),
                new GetAngleData(drive, angle).withTimeout(3));
    }

    public void handleFinish(Command command) {
        if (angle <= 180) {
            scheduler.schedule(createGroup());
        } else {
            SmartDashboard.putBoolean("Test/Done", true);
        }
    }

}