/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Unjam extends ParallelDeadlineGroup {
  /**
   * Creates a new Unjam.
   */
  public Unjam(ShooterAngle angle, Elevator elevator, Shooter shooter) {
    // Add your commands in the super() call. Add the deadline first.
    super(new InstantCommand(new Runnable() {
      @Override
      public void run() {
        angle.setAngle(0);
      }
    }), new Elevate(elevator), new Shoot(shooter));
  }
}
