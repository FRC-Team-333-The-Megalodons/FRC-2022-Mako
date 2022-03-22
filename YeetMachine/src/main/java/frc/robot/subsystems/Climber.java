// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Climber extends SubsystemBase {

  CANSparkMax leftClimber;
  CANSparkMax rightClimber;
  Joystick joystick;
  XboxController controller;
  DoubleSolenoid doubleClimbers;
  PneumaticHub hub;

  private final double CLIMB_SPEED = 1.0;

  /** Creates a new Climber. */
  public Climber(Joystick joystick_, XboxController controller_) {
    joystick = joystick_;
    controller = controller_;

    leftClimber = new CANSparkMax(Constants.DeviceIDs.LEFT_CLIMBER_ID, MotorType.kBrushless);
    rightClimber = new CANSparkMax(Constants.DeviceIDs.RIGHT_CLIMBER_ID, MotorType.kBrushless);

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    hub = new PneumaticHub(Constants.DeviceIDs.POWER_DISTRIBUTION_BOARD_PORT);

    doubleClimbers = hub.makeDoubleSolenoid(0, 1);//todo change this
  }

  public void climbUp() {
    leftClimber.set(CLIMB_SPEED);
    rightClimber.set(-CLIMB_SPEED);
  }

  public void climbDown() {
    leftClimber.set(-CLIMB_SPEED);
    rightClimber.set(CLIMB_SPEED);
  }

  public void stopClimb() {
    leftClimber.set(0.0);
    rightClimber.set(0.0);
  }

  public void doubleClimbersUp() {
    doubleClimbers.set(Value.kForward);
  }

  public void doubleClimbersDown() {
    doubleClimbers.set(Value.kReverse);
  }

  public boolean isRaiseClimberButtonPressed()
  {
    if (Constants.twoDriverMode)
    {
      return controller.getYButton();
    }
    else
    {
      return joystick.getRawButton(Constants.JoyStickButtons.CLIMBER_UP);
    }
  }

  public boolean isLowerClimberButtonPressed()
  {
    if (Constants.twoDriverMode)
    {
      return controller.getAButton();
    }
    else
    {
      return joystick.getRawButton(Constants.JoyStickButtons.CLIMBER_DOWN);
    }
  }

  //todo add as constants
  public boolean isDoubleClimberUpButtonPressed() {
    return controller.getRawButton(8);
  }

  public boolean isDoubleClimberDownButtonPressed(){
    return controller.getRawButton(7);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isRaiseClimberButtonPressed()) {
      climbUp();
    } else if (isLowerClimberButtonPressed()) {
      climbDown();
    } else {
      stopClimb();  
    }

    if(isDoubleClimberUpButtonPressed()){
      doubleClimbersUp();
    }

    if(isDoubleClimberDownButtonPressed()){
      doubleClimbersDown();
    }
  }  
}

