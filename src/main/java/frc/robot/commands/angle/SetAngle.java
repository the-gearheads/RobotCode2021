/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.angle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetAngle extends InstantCommand {
  private final ShooterAngle angle;
  private double target;
  private boolean target_set;

  public SetAngle(ShooterAngle angle) {
    this.angle = angle;
    SmartDashboard.putNumber("autoAngle", 70);
  }

  public SetAngle(ShooterAngle angle, double target) {
    this.angle = angle;
    this.target = target;
    target_set = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!target_set) {
      target = SmartDashboard.getNumber("autoAngle", 70);
    }
    angle.setAngle(target);
  }
}
