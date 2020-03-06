/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class BlockedElevate extends CommandBase {
  private final Elevator elevator;
  private final Intake intake;
  @Log
  private double setpoint;
  private int ballCount;
  @Log
  private double debug;
  private boolean last;

  public BlockedElevate(Elevator elevator, Intake intake) {
    this.elevator = elevator;
    this.intake = intake;
    this.setpoint = elevator.getLowerPosition();
    Logger.configureLoggingAndConfig(this, false);
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    ballCount = Shooter.getBallCount();
    elevator.zero();
  }

  @Override
  public void execute() {
    //TODO: if ball count is < 2, don't run top elavator motors.
    if (Shooter.topBlocked()) {
       elevator.lower(0);
       elevator.upper(0);
       intake.pft(0);
       return;
    }
    if (Shooter.bottomBlocked()) {
      elevator.lower(0.3);
      elevator.upper(0.25);
      intake.pft(0.4);
    } else {
      if (last) {
        setpoint = elevator.getLowerPosition() + Constants.SINGLE_BALL_ROTS;
      }
      if (elevator.getLowerPosition() < setpoint) {
        elevator.lower(0.3);
        elevator.upper(0.1);
        intake.pft(0.4);
      } else {
        elevator.lower(0);
        elevator.upper(0);
        intake.pft(0);
      }
    }
    last = Shooter.bottomBlocked();
  }

  @Override
  public void end(boolean interrupted) {
    elevator.lower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
