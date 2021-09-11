// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.NOP;
import frc.robot.commands.angle.HoldAngle;
import frc.robot.commands.angle.SetAngle;
import frc.robot.commands.drive.DriveToWall;
import frc.robot.commands.elevator.Elevate;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootAt;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.util.Subsystems;

public class KetteringAuton extends SequentialCommandGroup {
  /** Creates a new KetteringAuton. */
  public KetteringAuton(Subsystems s, double rpm, double angle, double direction) {
    /*
    1. Set angle to 
    2. Tune angle for 1 second
    3. Rev shooter at 6000 RPM for 1 second
    4. Shoot at 6000 for 4 seconds
    5. Wait for 0.5 seconds
    6. Drive forward for 1 seconds
    */
    super(new SetAngle(s.angle, angle), new HoldAngle(s.angle).withTimeout(1), new ShootAt(s.shooter).withTimeout(1),
        (new Shoot(s.shooter, rpm).alongWith((new Elevate(s.elevator)))).withTimeout(4),
        (new NOP().withTimeout(0.5)).andThen(new DriveToWall(s.drive, direction).withTimeout(1)));
  }
}

 