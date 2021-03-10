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

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
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
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.util.Deadband;
import frc.robot.util.Lidar;
import frc.robot.util.Tuple;
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

  private Lidar lidar;

  @Log
  private double lidarDistance;
  @Log
  private double angle;
  @Log
  private double angularVelocity;
  @Log
  private double x;
  @Log
  private double y;
  @Log
  private double xFeet;

  @Log
  private double leftVel;
  @Log
  private double rightVel;

  @Log 
  private double leftRotations;

  private double initAngle;
  private double speedMultiplier = 1;
  private double rotMultiplier = 1;


  private NetworkTableInstance inst; 
  private NetworkTable table;
  private NetworkTableEntry xEntry;
  private NetworkTableEntry yEntry;
  private NetworkTableEntry robotHeading;

  public DriveSubsystem() {
    Logger.configureLoggingAndConfig(this, false);
    lidar = new Lidar(Port.kMXP);

    this.inst = NetworkTableInstance.getDefault();
    this.table = inst.getTable("Live_Dashboard");
    this.xEntry = this.table.getEntry("robotX");
    this.yEntry = this.table.getEntry("robotY");
    this.robotHeading = this.table.getEntry("robotHeading");

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
    while (gyro.isCalibrating()) {
    }
    gyro.reset();
    gyro.zeroYaw();
    while (gyro.getAngle() == 0) {
    }
    initAngle = -gyro.getAngle();

    kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()),
        new Pose2d(0, 0, Rotation2d.fromDegrees(getAngle())));

    leftPosition = () -> -blMotor.getSelectedSensorPosition() * (ENCODER_CONSTANT);
    rightPosition = () -> -brMotor.getSelectedSensorPosition() * (-ENCODER_CONSTANT);
    leftVelocity = () -> blMotor.getSelectedSensorVelocity() * (ENCODER_CONSTANT * 10);
    rightVelocity = () -> brMotor.getSelectedSensorVelocity() * (-ENCODER_CONSTANT * 10);

    leftSide = new SpeedControllerGroup(flMotor, blMotor);
    rightSide = new SpeedControllerGroup(frMotor, brMotor);
    rightSide.setInverted(true);

    drive = new DifferentialDrive(leftSide, rightSide);
    drive.setSafetyEnabled(false); // disable auto-shutoff of motors... wpilib why??????
    drive.setDeadband(0);
    setSafety(true);

    setDefaultCommand(new ArcadeDrive(this));
  }

  public double getAngularVelocity() {
    return gyro.getRawGyroZ();
  }

  public void setSafety(boolean state) {
    flMotor.setSafetyEnabled(state);
    frMotor.setSafetyEnabled(state);
    blMotor.setSafetyEnabled(state);
    brMotor.setSafetyEnabled(state);
  }

  public double getAngle() {
    return (-gyro.getAngle()) - initAngle;
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

  public double getLidarDistance() {
    return lidarDistance;
  }

  @Override
  public void periodic() {
    leftVel = leftVelocity.get();
    rightVel = rightVelocity.get();
    angularVelocity = getAngularVelocity();
    angle = getAngle();
    Rotation2d gyroAngle = Rotation2d.fromDegrees(angle);
    Pose2d pose = odometry.update(gyroAngle, -leftPosition.get(), -rightPosition.get());
    x = pose.getTranslation().getX();
    y = pose.getTranslation().getY();
    lidarDistance = lidar.getDistance(false);

    xFeet = Units.metersToFeet(x);

    leftRotations = this.blMotor.getSelectedSensorPosition();

    this.xEntry.setNumber(x + 17);
    this.yEntry.setNumber(y + 12);
    this.robotHeading.setNumber(Units.degreesToRadians(angle));
  }

  public class Control {
    public final PIDController leftPid;
    public final PIDController rightPid;
    final PIDController gyroPid;

    final SimpleMotorFeedforward leftFF;
    final SimpleMotorFeedforward rightFF;

    private Control() {
      leftPid = new PIDController(Constants.LEFT_P, 0, 0);
      rightPid = new PIDController(Constants.RIGHT_P, 0, 0);
      gyroPid = new PIDController(Constants.GYRO_P, 0, 0);

      leftFF = Constants.leftFF;
      rightFF = Constants.rightFF;
    }

    public void setMultipliers(double drive, double rot) {
      speedMultiplier = drive;
      rotMultiplier = rot;
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

    public void driveVoltageFF(Tuple voltages, DifferentialDriveWheelSpeeds speeds) {
      leftSide.setVoltage(voltages.left + leftFF.calculate(speeds.leftMetersPerSecond));
      rightSide.setVoltage(voltages.right + rightFF.calculate(speeds.rightMetersPerSecond));
    }

    public double angleFeedForward(double input) {
      double degs = Math.toDegrees(input);
      if (Math.abs(degs) < 1) {
        return 0;
      }
      return Math.toRadians(degs + Math.copySign(41.86, input));
    }

    public ChassisSpeeds gyroLoop(ChassisSpeeds chs) {
      double angVelRads = Math.toRadians(getAngularVelocity());
      double effort = gyroPid.calculate(angVelRads, chs.omegaRadiansPerSecond);
      if (Deadband.get(angVelRads, Math.toRadians(3), 0) == 0) {
        effort = 0;
      }
      return new ChassisSpeeds(chs.vxMetersPerSecond, chs.vyMetersPerSecond,
          angleFeedForward(chs.omegaRadiansPerSecond) + effort);
    }

    public void arcadeDrive(ChassisSpeeds chs) {
      ChassisSpeeds chs2 = new ChassisSpeeds(chs.vxMetersPerSecond * speedMultiplier,
          chs.vyMetersPerSecond * speedMultiplier,
          Math.toRadians(Math.toDegrees(chs.omegaRadiansPerSecond) * rotMultiplier));
      DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(gyroLoop(chs2));
      tankDrive(speeds);
    }

    public void tankDrive(DifferentialDriveWheelSpeeds speeds) {
      Tuple voltages = new Tuple(leftPid.calculate(leftVelocity.get(), speeds.leftMetersPerSecond),
          rightPid.calculate(rightVelocity.get(), speeds.rightMetersPerSecond));
      driveVoltageFF(voltages, speeds);
    }

  }
}
