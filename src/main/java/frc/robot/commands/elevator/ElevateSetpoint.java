// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevateSetpoint extends CommandBase {
  /** Creates a new ElevateSetpoint. */
  private final Elevator elevator;
  private final double setpoint;
  public ElevateSetpoint(Elevator elevator, double setpoint) {
    this.elevator = elevator;
    this.setpoint = elevator.getLowerPosition() + setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (setpoint > elevator.getLowerPosition()) {
      elevator.elevate();
    } else {
      elevator.zero();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return setpoint < elevator.getLowerPosition();
  }
}
