// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ControllerButtons;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  CANSparkMax motor1 = new CANSparkMax(ShooterConstants.IndexerConstants.motor1,MotorType.kBrushed);
  CANSparkMax motor2 = new CANSparkMax(ShooterConstants.IndexerConstants.motor2,MotorType.kBrushed);
  Joystick stick = new Joystick(Constants.DeviceIDs.JOYSTICK_PORT);
  XboxController controller = new XboxController(Constants.DeviceIDs.CONTROLLER_PORT);

  public Indexer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(stick.getTrigger()){
      motor2.set(.6);
    }
    if(controller.getRawButton(ControllerButtons.RIGHT_TRIGGER)){
      motor1.set(.7);
    }
  }
}
