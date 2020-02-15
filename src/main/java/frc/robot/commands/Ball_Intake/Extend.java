/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Ball_Intake;

import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extender;

public class Extend extends CommandBase {
  private Extender extender;

  /**
   * Creates a new Extender.
   */
  public Extend(Extender extender) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(extender);
    this.extender = extender;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extender.extend(0.5);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extender.extend(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (extender.getPosition()>= Units.inchesToMeters(9));
  }
}
