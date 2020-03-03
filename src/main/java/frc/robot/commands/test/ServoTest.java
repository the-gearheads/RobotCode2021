/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class ServoTest extends CommandBase {
  /**jj
   * Creates a new ServoTest.
   */
  private Servo servo;
  @Log
  private double servoAngle;
  public ServoTest() {
    servo = new Servo(0);
    // Use addRequirements() here to declare subsystem dependencies.
    Logger.configureLoggingAndConfig(this, false);
  }

  /**
   * @param servoAngle the servoAngle to set
   */
  @Config
  public void setServoAngle(double servoAngle) {
    this.servoAngle = servoAngle;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    servo.setAngle(servoAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
