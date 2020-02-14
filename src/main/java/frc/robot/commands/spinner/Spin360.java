/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;

public class Spin360 extends CommandBase {
  private static Spinner spinner;
  private String lastColor;
  private String targetColor;
  private int spinCount;
  private int rotations;

  /**
   * Creates a new Spin360.
   */
  public Spin360(Spinner subsystem, int rot) {
    addRequirements(subsystem);
    spinner = subsystem;
    rotations = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetColor = spinner.getColor();
    lastColor = spinner.getColor();
    spinCount = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String currentColor = spinner.getColor();
    if (currentColor != lastColor) {
      lastColor = currentColor;
      if (currentColor == targetColor) {
        spinCount += 1;
      }
    }
    SmartDashboard.putNumber("spinCount", spinCount);
    SmartDashboard.putString("Color:", currentColor);
    spinner.spinClockwise();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    spinner.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (spinCount >= rotations * 2);
  }
}
