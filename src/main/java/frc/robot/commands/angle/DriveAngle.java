/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.angle;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.util.Deadband;

public class DriveAngle extends CommandBase {
  private final ShooterAngle angle;

  public DriveAngle(ShooterAngle angle) {
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Deadband.getSmart(-RobotContainer.joystick.getRawAxis(1), 0.1);
    angle.turnAngle(speed / 5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angle.setSetpoint(angle.getPosition());
    // angle.turnAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
