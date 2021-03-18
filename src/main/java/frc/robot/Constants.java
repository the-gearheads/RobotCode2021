/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.profile.AkhilDrive;
import frc.robot.profile.DriverProfile;

public final class Constants {

    //// PROFILE SELECT
    public static final DriverProfile profile = new AkhilDrive();

    //// MOTOR CONTROLLER IDs
    public static final int FL_ID = 3;
    public static final int FR_ID = 5;
    public static final int BL_ID = 2;
    public static final int BR_ID = 6;

    ////

    public static double THROTTLE_DEADBAND = 0.05;
    public static double ROT_DEADBAND = 0.3;

    ////

    //// OPERATOR CONFIG

    // degrees per second
    public static double ANGLE_DRIVE_SPEED = 15;
    public static double RPM_MIN = 5000;
    public static double RPM_MAX = 8000;

    ////

    //// DRIVETRAIN SETTINGS

    // Kinematics and Odometry
    public static final double TRACK_WIDTH = .68; // m
    public static final double GEARING = 12.75; // gear ratio
    public static final double WHEEL_DIAMETER = .195; // m
    public static final double ENCODER_EPR = 2048; // encoder units
    public static final double MAX_VELOCITY = 1.5; // m/s
    public static final double MAX_ACCEL = 3; // m/s²
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;

    // Feed Forward
    public static final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0.46, 1.84, 0.193);
    public static final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0.5, 1.82, 0.185);

    // PID Config
    public static final double LEFT_P = 0; // .000425;
    public static final double RIGHT_P = 0; // .000155;
    public static final double ANGLE_P = 0;

    ////
    public static final double LEFT_DISTANCE = 25.07;
    public static final double RIGHT_DISTANCE = 24.93;


    //// SHOOTER SETTINGS

    // Main shooter
    public static final SimpleMotorFeedforward SHOOTER_FF_LEFT = new SimpleMotorFeedforward(0.113, 0.0622, 0.00721);
    public static final SimpleMotorFeedforward SHOOTER_FF_RIGHT = new SimpleMotorFeedforward(0.124, 0.063, 0.00566);
    public static final int SHOOTER_POT = 0;
    public static final int IR_TOP = 2;
    public static final int IR_BOTTOM = 3;
    public static final int RANGE_FINDER = 1;
    public static final int SHOOTER_GEARING = 1;
    public static final double SHOOTER_P = .02;
    public static final double SHOOTER_D = .005;

    // Shooter angle
    public static final double SHOOTER_ANGLE_MIN = 2.8;
    public static final double SHOOTER_ANGLE_MAX = 66.5;

    ////

    //// ELEVATOR SETTINGS

    public static final double SINGLE_BALL_ROTS = 1.5;

    ////

    //// GYRO SETTINGS

    public static final Port GYRO_PORT = Port.kMXP;
    public static final double GYRO_P = 1;

    //// WINCH SETTINGS
    public static final int WINCH_ROTATIONS = 10;
    ////

    //// INTAKE SETTINGS
    public static final double INTAKE_EPR = 42;
    public static final double INTAKE_GEARING = 4;
    public static final double LEAD_SCREW = 1;
    ////

    //// MISC

    public static final int CONTROLLER_PORT = 0;
    public static final int JOYSTICK_PORT = 1;

    ////

}
