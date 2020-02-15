/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  private final CANSparkMax rShooter;
  private final CANSparkMax lShooter;
  private final CANSparkMax elevatorT;
  private final CANSparkMax elevatorB;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    rShooter = new CANSparkMax(0,MotorType.kBrushless);
    lShooter = new CANSparkMax(0,MotorType.kBrushless);
    elevatorT = new CANSparkMax(0,MotorType.kBrushless);
    elevatorB = new CANSparkMax(0,MotorType.kBrushless);
  }

  public void elevator(double speed) {
    elevatorT.set(speed);
    elevatorB.set(-speed);

  }
  public void shot(double speed) {
    rShooter.set(speed);
    lShooter.set(-speed);

  }
}

