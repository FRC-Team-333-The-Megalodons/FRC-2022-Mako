// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  CANSparkMax motor1 = new CANSparkMax(ShooterConstants.IndexerConstants.motor1,MotorType.kBrushed);
  CANSparkMax motor2 = new CANSparkMax(ShooterConstants.IndexerConstants.motor2,MotorType.kBrushed);
  Joystick stick = new Joystick(0);

  public Indexer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
