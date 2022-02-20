// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  CANSparkMax intakeMotor;

  Joystick joystick;

  Solenoid intakSolenoid;

  public Intake() {

    intakeMotor = new CANSparkMax(Constants.DeviceIDs.INTAKE_MOTOR_ID,MotorType.kBrushed);
    //intakeMotor.setInverted(false);

    joystick = new Joystick(Constants.DeviceIDs.JOYSTICK_PORT);

    //intakSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  }

  public void runIntake(){
    //intakeMotor.setInverted(false);
    intakeMotor.set(0.75);
    //intakSolenoid.set(true);
  }

  public void runIntakeReverse(){
    //intakeMotor.setInverted(true);
    intakeMotor.set(-0.75);
    //motor.set(-1);
  }

  public void stopIntake() {
    intakeMotor.set(0);
    //intakSolenoid.set(false);
    //motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(joystick.getRawButton(Constants.JoyStickButtons.INTAKE_RUN)){
      runIntake();
      System.out.println("***");
    }else if(joystick.getRawButton(Constants.JoyStickButtons.INTAKE_REVERSE)){
      runIntakeReverse();
      System.out.println("&&&&0");
    } else {
      stopIntake();
    }
  }
}
