// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.element.Element;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.dashboard.SmartDashboardWrapper;
import frc.robot.utils.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  CANSparkMax intakeMotor;
  CANSparkMax holderMotor;

  Joystick joystick;
  XboxController controller;

  PneumaticHub hub;

  DoubleSolenoid intakeSols;
  SmartDashboardWrapper dashboard;

  private final double INTAKE_SPEED_DEFAULT = 0.65;
  private double INTAKE_SPEED = INTAKE_SPEED_DEFAULT;
  private final String INTAKE_SPEED_KEY = "Intake Speed";

  private final double HOLDER_SPEED_DEFAULT = 0.85;
  private double HOLDER_SPEED = HOLDER_SPEED_DEFAULT;
  private final String HOLDER_SPEED_KEY = "Holder Speed";

  public Intake() {

    intakeMotor = new CANSparkMax(Constants.DeviceIDs.INTAKE_MOTOR_ID,MotorType.kBrushed);
    holderMotor = new CANSparkMax(Constants.DeviceIDs.HOLDER_ID,MotorType.kBrushed);//TODO do constant
    //intakeMotor.setInverted(false);

    joystick = new Joystick(Constants.DeviceIDs.JOYSTICK_PORT);
    controller = new XboxController(Constants.DeviceIDs.CONTROLLER_PORT);;

    //intakSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.DeviceIDs.INTAKE_SOLENOID);

    hub = new PneumaticHub(Constants.DeviceIDs.PNEMATIC_HUB);

    intakeSols = hub.makeDoubleSolenoid(Constants.DeviceIDs.INTAKE_SOLENOID1, Constants.DeviceIDs.INTAKE_SOLENOID2);

    
    dashboard = new SmartDashboardWrapper(this);
    dashboard.putNumber(INTAKE_SPEED_KEY, INTAKE_SPEED);
    dashboard.putNumber(HOLDER_SPEED_KEY, HOLDER_SPEED);
  }

  public void runIntake(){
    //intakeMotor.setInverted(false);
    intakeMotor.set(-INTAKE_SPEED);
    //intakSolenoid.set(true);
  }

  public void runIntakeReverse(){
    //intakeMotor.setInverted(true);
    intakeMotor.set(INTAKE_SPEED);
    //motor.set(-1);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void stopHolder() {
    holderMotor.set(0);
  }

  public void runHolder(){
    holderMotor.set(HOLDER_SPEED);
  }

  public void runHolderReverse(){
    holderMotor.set(-HOLDER_SPEED);
  }

  public void intakeOut() {
    intakeSols.set(Value.kForward);
  }

  public void intakeIn() {
    intakeSols.set(Value.kReverse);;
  }

  @Override
  public void periodic() {
    INTAKE_SPEED = dashboard.getNumber(INTAKE_SPEED_KEY, INTAKE_SPEED_DEFAULT);
    HOLDER_SPEED = dashboard.getNumber(HOLDER_SPEED_KEY, HOLDER_SPEED_DEFAULT);
    // This method will be called once per scheduler run
    if((Constants.twoDriverMode && controller.getRightBumper()) || (joystick.getRawButton(Constants.JoyStickButtons.INTAKE_RUN) && !Constants.twoDriverMode)){//joystick.getRawButton(Constants.JoyStickButtons.INTAKE_RUN
      runIntake();
      runHolder();
      //System.out.println("intaking balls");
    }else if((Constants.twoDriverMode && controller.getLeftTriggerAxis() > 0) || (joystick.getRawButton(Constants.JoyStickButtons.INTAKE_REVERSE) && !Constants.twoDriverMode)){//joystick.getRawButton(Constants.JoyStickButtons.INTAKE_REVERSE
      runIntakeReverse();
      runHolderReverse();
      //System.out.println("intake back");
    } else {
      stopIntake();
      stopHolder();
    }
    
    if ((Constants.twoDriverMode && controller.getAButton()) || (joystick.getRawButton(Constants.JoyStickButtons.INTAKE_FORWARD) && Constants.twoDriverMode)) {
      intakeOut();
      //System.out.println("intake sols forward");
    }
    if((Constants.twoDriverMode && controller.getBButton()) || (joystick.getRawButton(Constants.JoyStickButtons.INTAKE_BACK) && Constants.twoDriverMode)) {
      intakeIn();
      //System.out.println("intake sols back"); 0
      
    }
  }
}
