// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class WaitElevate extends CommandBase {

  private final Intake intake;
  private final Elevator elevator;
  private final int ballsToIntake;
  private double setpoint;
  //@Log
  private int intakes;

  public WaitElevate(Elevator elevator, Intake intake, int ballsToIntake) {
    this.elevator = elevator;
    this.intake = intake;
    this.ballsToIntake = ballsToIntake;

    //Logger.configureLoggingAndConfig(this, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakes = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getIntook()) {
      intakes += 1;
      double difference = (intakes > 1 ? (1d/3d) : 1.0);
      setpoint = elevator.getLowerPosition() + (Constants.SINGLE_BALL_ROTS * difference);
    }
    if (setpoint > elevator.getLowerPosition()) {
      elevator.elevate();
    } else {
      elevator.zero();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakes >= ballsToIntake && setpoint < elevator.getLowerPosition();
  }
}
