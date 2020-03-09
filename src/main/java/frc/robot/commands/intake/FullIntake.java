/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class FullIntake extends CommandBase {
  private Intake intake;

  public FullIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake.pft(.5);
    intake.intake(.5);
  }

  @Override
  public void end(boolean interrupted) {
    intake.pft(0);
    intake.intake(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
