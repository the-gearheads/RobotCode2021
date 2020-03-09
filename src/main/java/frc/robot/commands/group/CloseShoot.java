/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.angle.HoldAngle;
import frc.robot.commands.angle.SetAngle;
import frc.robot.commands.drive.TurnToAngle;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.shooter.ShootAt;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CloseShoot extends SequentialCommandGroup {
  /**
   * Creates a new CloseShoot.
   */
  public CloseShoot(DriveSubsystem drive, Shooter shooter, ShooterAngle angle, Elevator elevator) {
    // super(new SetAngle(angle, 28), (new HoldAngle(angle)).withTimeout(1),
        // new ShootAt(shooter, 5000).withTimeout(.5), (new ShootAt(shooter, 5000).alongWith(new Elevate(elevator))));
  }
}
