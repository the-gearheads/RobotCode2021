/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class Elevator extends CommandBase {
  private Shooter shooterElevator; 
  /**
   * Creates a new elevator.
   */
  public Elevator(Shooter shooterElevator) {
    addRequirements(shooterElevator);
    this.shooterElevator = shooterElevator;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upperSpeed = SmartDashboard.getNumber("upperSpeed", 0);
    double lowerSpeed = SmartDashboard.getNumber("lowerSpeed", 0);
    shooterElevator.elevator(upperSpeed, lowerSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterElevator.elevator(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
