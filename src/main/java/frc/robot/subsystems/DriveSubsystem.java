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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.util.Deadband;
import frc.robot.util.WheelVoltages;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase {

  private final double ENCODER_CONSTANT = (1 / (double) Constants.ENCODER_EPR) * (1 / (double) Constants.GEARING)
      * Constants.WHEEL_DIAMETER * Math.PI;
  private final WPI_TalonFX flMotor;
  private final WPI_TalonFX frMotor;
  private final WPI_TalonFX blMotor;
  private final WPI_TalonFX brMotor;
  private final SpeedControllerGroup leftSide;
  private final SpeedControllerGroup rightSide;

  private final DifferentialDrive drive;
  public final DifferentialDriveKinematics kinematics;
  private final DifferentialDriveOdometry odometry;

  private final AHRS gyro;

  private Supplier<Double> leftPosition;
  private Supplier<Double> rightPosition;
  private Supplier<Double> leftVelocity;
  private Supplier<Double> rightVelocity;

  public final Control controller = new Control();

  // Log vars
  @Log
  double angle;
  @Log
  double angularVelocity;
  @Log
  double x;
  @Log
  double y;

  public DriveSubsystem() {
    Logger.configureLoggingAndConfig(this, false);

    // Setup motors
    flMotor = new WPI_TalonFX(Constants.FL_ID);
    frMotor = new WPI_TalonFX(Constants.FR_ID);
    blMotor = new WPI_TalonFX(Constants.BL_ID);
    brMotor = new WPI_TalonFX(Constants.BR_ID);

    // Setup encoders
    blMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
    brMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
    blMotor.setSelectedSensorPosition(0);
    brMotor.setSelectedSensorPosition(0);

    // Set up followers and braking mode
    flMotor.follow(blMotor);
    frMotor.follow(brMotor);

    flMotor.setNeutralMode(NeutralMode.Brake);
    frMotor.setNeutralMode(NeutralMode.Brake);
    blMotor.setNeutralMode(NeutralMode.Brake);
    brMotor.setNeutralMode(NeutralMode.Brake);

    gyro = new AHRS(Constants.GYRO_PORT);
    kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()), new Pose2d(5.5, 13.5, Rotation2d.fromDegrees(getAngle())));

    leftPosition = () -> blMotor.getSelectedSensorPosition() * (ENCODER_CONSTANT);
    rightPosition = () -> brMotor.getSelectedSensorPosition() * (-ENCODER_CONSTANT);
    leftVelocity = () -> blMotor.getSelectedSensorVelocity() * (ENCODER_CONSTANT * 10);
    rightVelocity = () -> brMotor.getSelectedSensorVelocity() * (-ENCODER_CONSTANT * 10);

    leftSide = new SpeedControllerGroup(flMotor, blMotor);
    rightSide = new SpeedControllerGroup(frMotor, brMotor);
    rightSide.setInverted(true);

    drive = new DifferentialDrive(blMotor, brMotor);
    drive.setSafetyEnabled(false); // disable auto-shutoff of motors... wpilib why??????
    drive.setDeadband(0);

    setDefaultCommand(new ArcadeDrive(this));
  }

  public double getAngularVelocity() {
    return gyro.getRawGyroZ();
  }

  public double getAngle() {
    return -gyro.getAngle();
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftVelocity.get(), rightVelocity.get());
  }

  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
    angularVelocity = getAngularVelocity();
    angle = getAngle();
    Rotation2d gyroAngle = Rotation2d.fromDegrees(angle);
    Pose2d pose = odometry.update(gyroAngle, leftPosition.get(), rightPosition.get());
    x = pose.getTranslation().getX();
    y = pose.getTranslation().getY();
    table.getEntry("robotX").setNumber(x);
    table.getEntry("robotY").setNumber(y);
    table.getEntry("robotHeading").setNumber(Math.toRadians(angle));
  }

  public void tankDriveVolts(Double left, Double right) {
    controller.rawDriveVoltage(left, right);
  }

  public class Control {
    public final PIDController leftPid;
    public final PIDController rightPid;
    final PIDController gyroPid;

    final SimpleMotorFeedforward leftFF;
    final SimpleMotorFeedforward rightFF;

    private Control() {
      leftPid = new PIDController(Constants.LEFT_P, 0, Constants.LEFT_D);
      rightPid = new PIDController(Constants.RIGHT_P, 0, Constants.RIGHT_D);
      gyroPid = new PIDController(Constants.GYRO_P, 0, Constants.GYRO_D);

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

    public void rawDriveVoltage(double left, double right) {
      leftSide.setVoltage(left);
      rightSide.setVoltage(right);
    }

    public void driveVoltageFF(WheelVoltages voltages, DifferentialDriveWheelSpeeds speeds) {
      leftSide.setVoltage(voltages.left + leftFF.calculate(speeds.leftMetersPerSecond));
      rightSide.setVoltage(voltages.right + rightFF.calculate(speeds.rightMetersPerSecond));
    }

    public void driveVoltageFF(WheelVoltages voltages, DifferentialDriveWheelSpeeds speeds,
        DifferentialDriveWheelSpeeds prevSpeeds, double dt) {
      double leftFeedforward = leftFF.calculate(speeds.leftMetersPerSecond,
          (speeds.leftMetersPerSecond - prevSpeeds.leftMetersPerSecond) / dt);
      double rightFeedforward = rightFF.calculate(speeds.rightMetersPerSecond,
          (speeds.rightMetersPerSecond - prevSpeeds.rightMetersPerSecond) / dt);

      leftSide.setVoltage(voltages.left + leftFeedforward);
      rightSide.setVoltage(voltages.right + rightFeedforward);
    }

    public double angleFeedForward(double input) {
      double degs = Math.toDegrees(input);
      return Deadband.get(Math.toRadians(degs + Math.copySign(30, input)), 5);
    }

    public ChassisSpeeds gyroLoop(ChassisSpeeds chs) {
      double angVelRads = Math.toRadians(getAngularVelocity());
      double effort = gyroPid.calculate(angVelRads, chs.omegaRadiansPerSecond);
      return new ChassisSpeeds(chs.vxMetersPerSecond, chs.vyMetersPerSecond,
          angleFeedForward(chs.omegaRadiansPerSecond) + effort);
    }

    public void arcadeDrive(ChassisSpeeds chs) {
      DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(gyroLoop(chs));
      tankDrive(speeds);
    }

    public void tankDrive(DifferentialDriveWheelSpeeds speeds) {
      WheelVoltages voltages = new WheelVoltages(leftPid.calculate(leftVelocity.get(), speeds.leftMetersPerSecond),
          rightPid.calculate(rightVelocity.get(), speeds.rightMetersPerSecond));
      driveVoltageFF(voltages, speeds);
    }

    public void tankDrive(DifferentialDriveWheelSpeeds speeds, DifferentialDriveWheelSpeeds prevSpeeds, double dt) {
      WheelVoltages voltages = new WheelVoltages(leftPid.calculate(leftVelocity.get(), speeds.leftMetersPerSecond),
          rightPid.calculate(rightVelocity.get(), speeds.rightMetersPerSecond));
      driveVoltageFF(voltages, speeds, prevSpeeds, dt);
    }

  }
}