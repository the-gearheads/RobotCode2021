/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Pft extends CommandBase {
  private Intake intake;
  private boolean reverse;

  public Pft(Intake intake, boolean reverse) {
    this.intake = intake;
    this.reverse = reverse;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (reverse) {
      intake.pft(-.5);
    }
    else {
      intake.pft(.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.pft(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
