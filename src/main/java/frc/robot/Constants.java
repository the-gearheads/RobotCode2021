/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public final class Constants {

    // MOTOR CONTROLLER IDs
    public static final int FL_ID = 1;
    public static final int FR_ID = 5;
    public static final int BL_ID = 2;
    public static final int BR_ID = 6;

    //// DRIVING CONFIG

    // Meters per second
    public static double THROTTLE_SPEED = 1;
    // Degrees per second
    public static double ROT_SPEED = 90;

    // Controller deadband in % of stick
    public static double THROTTLE_DEADBAND = 0.05;
    public static double ROT_DEADBAND = 0.3;

    ////

    //// DRIVETRAIN SETTINGS

    // Kinematics and Odometry
    public static final double TRACK_WIDTH = .68; // m
    public static final double GEARING = 10.71; // gear ratio
    public static final double WHEEL_DIAMETER = .2032; // m
    public static final double ENCODER_EPR = 2048; // encoder units
    public static final double MAX_VELOCITY = 6; // m/s
    public static final double MAX_ACCEL = 3; // m/s²

    // Feed Forward
    public static final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0.46, 1.84, 0.193);
    public static final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0.5, 1.82, 0.185);

    // PID Config
    public static final double LEFT_P = 0; // .000425;
    public static final double LEFT_D = 0;
    public static final double RIGHT_P = 0; // .000155;
    public static final double RIGHT_D = 0;

    ////

    //// GYRO
    public static final Port GYRO_PORT = Port.kMXP;
    public static final double GYRO_P = 1;
    public static final double GYRO_D = 0;

    //// MISC

    public static final int CONTROLLER_PORT = 0;

    ////

}
