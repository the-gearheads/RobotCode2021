/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.NOP;
import frc.robot.commands.angle.HoldAngle;
import frc.robot.commands.angle.SetAngle;
import frc.robot.commands.drive.DriveToWall;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.shooter.ShootAt;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class MilfordAuton extends SequentialCommandGroup {
  /**
   * Creates a new MilfordAuton.
   */
  public MilfordAuton(DriveSubsystem drive, Shooter shooter, ShooterAngle angle, Elevator elevator) {
    /*
    1. Set angle to 45
    2. Tune angle for 1 second
    3. Rev shooter at 6000 RPM for 1 second
    4. Shoot at 6000 for 4 seconds
    5. Wait for 0.5 seconds
    6. Drive forward for 3 seconds
    */
    super(new SetAngle(angle, 45), new HoldAngle(angle).withTimeout(1), new ShootAt(shooter, 5750).withTimeout(1),
        (new ShootAt(shooter, 5750).alongWith((new Elevate(elevator)))).withTimeout(4),
        (new NOP().withTimeout(0.5)).andThen(new DriveToWall(drive).withTimeout(3)));
  }
}
