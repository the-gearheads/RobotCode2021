/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonFX flMotor;
  private final WPI_TalonFX frMotor;
  private final WPI_TalonFX blMotor;
  private final WPI_TalonFX brMotor;
  private final SpeedControllerGroup leftSide;
  private final SpeedControllerGroup rightSide;
  private final DifferentialDrive drive;

  public DriveSubsystem() {
    flMotor = new WPI_TalonFX(Constants.FL_ID);
    frMotor = new WPI_TalonFX(Constants.FR_ID);
    blMotor = new WPI_TalonFX(Constants.BL_ID);
    brMotor = new WPI_TalonFX(Constants.BR_ID);
    leftSide = new SpeedControllerGroup(flMotor, blMotor);
    rightSide = new SpeedControllerGroup(frMotor, brMotor);
    drive = new DifferentialDrive(leftSide, rightSide);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }
}
