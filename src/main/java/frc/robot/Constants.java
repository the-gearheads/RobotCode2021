/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public final class Constants {

    // MOTOR CONTROLLER IDs
    public static final int FL_ID = 1;
    public static final int FR_ID = 5;
    public static final int BL_ID = 2;
    public static final int BR_ID = 6;

    //// DRIVER CONFIG

    // Final Speed = MAX_SPEED*SPEED_MULTIPLIER

    // Meters per second
    public static double THROTTLE_SPEED = 2.5;
    public static double ROT_SPEED = 0.5;

    ////

    //// DRIVETRAIN SETTINGS

    // Kinematics Measurements
    public static final double TRACK_WIDTH = 7.86;
    public static final double GEARING = 10.71;
    public static final double WHEEL_DIAMETER = .2032;
    public static final double ENCODER_EPR = 2048;

    // Feed Forward
    public static final SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0.46, 1.84, 0.193);
    public static final SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0.5, 1.82, 0.185);

    // PID Config
    public static final double LEFT_P = 0;//.000425;
    public static final double LEFT_D = 0;
    public static final double RIGHT_P = 0;//.000155;
    public static final double RIGHT_D = 0;

    ////

    //// MISC

    public static final int CONTROLLER_PORT = 0;

    ////

}
