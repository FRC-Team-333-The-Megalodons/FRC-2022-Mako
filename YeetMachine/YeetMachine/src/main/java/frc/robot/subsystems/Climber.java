// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Climber extends SubsystemBase {

  CANSparkMax leftClimber;
  CANSparkMax rightClimber;
  Joystick joystick;

  private final double CLIMB_SPEED = 0.5;

  /** Creates a new Climber. */
  public Climber() {

    

    leftClimber = new CANSparkMax(Constants.DeviceIDs.LEFT_CLIMBER_ID, MotorType.kBrushless);
    rightClimber = new CANSparkMax(Constants.DeviceIDs.RIGHT_CLIMBER_ID, MotorType.kBrushless);
  }

  public void climbUp() {
    leftClimber.set(CLIMB_SPEED);
  }

  public void climbDown() {
    leftClimber.set(-CLIMB_SPEED);
    rightClimber.set(-CLIMB_SPEED);
  }

  public void stopClimb() {
    leftClimber.set(0.0);
    rightClimber.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    if(joystick.getRawButton(Constants.JoyStickButtons.CLIMBER_UP)) {
      climbUp();
    } else if (joystick.getRawButton(Constants.JoyStickButtons.CLIMBER_DOWN)) {
      climbDown();
    } else {
      */
  }
}

