/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.util.WheelVoltages;

public class DriveSubsystem extends SubsystemBase {

  private final double ENCODER_CONSTANT = (1 / (double) Constants.ENCODER_EPR) * (1 / Constants.GEARING)
      * Constants.WHEEL_DIAMETER * Math.PI;
  private final WPI_TalonFX flMotor;
  private final WPI_TalonFX frMotor;
  private final WPI_TalonFX blMotor;
  private final WPI_TalonFX brMotor;
  private final SpeedControllerGroup leftSide;
  private final SpeedControllerGroup rightSide;
  private final DifferentialDrive drive;
  private final DifferentialDriveKinematics kinematics;

  private Supplier<Double> leftVelocity;
  private Supplier<Double> rightVelocity;

  public final Control controller = new Control();

  public DriveSubsystem() {
    flMotor = new WPI_TalonFX(Constants.FL_ID);
    frMotor = new WPI_TalonFX(Constants.FR_ID);
    blMotor = new WPI_TalonFX(Constants.BL_ID);

    blMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
    blMotor.setSelectedSensorPosition(0);
    brMotor = new WPI_TalonFX(Constants.BR_ID);
    brMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
    brMotor.setSelectedSensorPosition(0);

    flMotor.follow(blMotor);
    frMotor.follow(brMotor);

    flMotor.setNeutralMode(NeutralMode.Brake);
    frMotor.setNeutralMode(NeutralMode.Brake);
    blMotor.setNeutralMode(NeutralMode.Brake);
    brMotor.setNeutralMode(NeutralMode.Brake);

    kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    leftVelocity = () -> blMotor.getSelectedSensorVelocity() * (ENCODER_CONSTANT * 10);
    rightVelocity = () -> brMotor.getSelectedSensorVelocity() * (-ENCODER_CONSTANT * 10);

    leftSide = new SpeedControllerGroup(flMotor, blMotor);
    rightSide = new SpeedControllerGroup(frMotor, brMotor);
    rightSide.setInverted(true);

    drive = new DifferentialDrive(flMotor, frMotor);
    drive.setSafetyEnabled(false); // disable auto-shutoff of motors... wpilib why??????
    drive.setDeadband(0);

    setDefaultCommand(new ArcadeDrive(this));
  }

  /*
   * NAMING SCHEME: FF = Feedforward without PID raw = Does not use PID or
   * feedforward no prefix = uses PID and feedforward
   */
  public class Control {
    final PIDController leftPid;
    final PIDController rightPid;

    final SimpleMotorFeedforward leftFF;
    final SimpleMotorFeedforward rightFF;

    private Control() {
      leftPid = new PIDController(Constants.LEFT_P, 0, Constants.LEFT_D);
      rightPid = new PIDController(Constants.RIGHT_P, 0, Constants.RIGHT_D);

      leftFF = Constants.leftFF;
      rightFF = Constants.rightFF;
    }

    public void rawDrive(double left, double right) {
      leftSide.set(left);
      rightSide.set(right);
    }

    public void rawTankDrive(double leftSpeed, double rightSpeed) {
      drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void rawArcadeDrive(double speed, double rotation) {
      drive.arcadeDrive(speed, rotation);
    }

    public void driveVoltageFF(WheelVoltages voltages, DifferentialDriveWheelSpeeds speeds) {
      leftSide.setVoltage(voltages.left + leftFF.calculate(speeds.leftMetersPerSecond));
      rightSide.setVoltage(voltages.right + rightFF.calculate(speeds.rightMetersPerSecond));
    }

    public void arcadeDrive(ChassisSpeeds chs) {
      DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(chs);
      tankDrive(speeds);
    }

    public void tankDrive(DifferentialDriveWheelSpeeds speeds) {
      WheelVoltages voltages = new WheelVoltages(leftPid.calculate(leftVelocity.get(), speeds.leftMetersPerSecond),
          rightPid.calculate(rightVelocity.get(), speeds.rightMetersPerSecond));
      driveVoltageFF(voltages, speeds);
    }

  }
}