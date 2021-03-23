/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
  @Log
  private double debug;
  private boolean needToIntake;
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
    elevator.zero();
    elevator.setUpperCoast(true);
  }

  @Override
  public void execute() {
    // if (needToIntake) {
    //   if (Shooter.bottomBlocked()) {
    //     needToIntake = false;
    //     setpoint = elevator.getLowerPosition() + Constants.SINGLE_BALL_ROTS;
    //   }
    //   elevator.elevate(0, .2);
    //   return;
    // }
    if (Shooter.getBallCount() > 2) {
      elevator.setUpperCoast(false);
    }
    needToIntake = intake.getIntook();
    boolean noTop = Shooter.topBlocked();
    if (Shooter.bottomBlocked()) {
      elevator.elevateLower();
      if (!noTop) {
        elevator.elevateUpper();
      }
    } else {
      if (last) {
        setpoint = elevator.getLowerPosition() + 5;
      }
      if (elevator.getLowerPosition() < setpoint) {
        elevator.elevateLower();
        if (!noTop) {
          elevator.elevateUpper();
        }
      } else {
        elevator.stop();
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
