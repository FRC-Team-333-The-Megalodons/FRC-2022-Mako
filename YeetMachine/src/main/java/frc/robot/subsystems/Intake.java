// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.element.Element;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
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
import frc.robot.subsystems.LimitSwitch.TimedLimitSwitch;
import frc.robot.utils.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  CANSparkMax intakeMotor;

  Joystick joystick;
  XboxController controller;

  PneumaticHub hub;

  DoubleSolenoid intakeSols;
  SmartDashboardWrapper dashboard;
  LimitSwitch catapultLimitSwitch;
  TimedLimitSwitch intakeLimitSwitch;

  private final double NOMINAL_SPEED = 0.333;
  private final double INTAKE_SPEED_DEFAULT = 0.5;
  private double INTAKE_SPEED = INTAKE_SPEED_DEFAULT;
  private final String INTAKE_SPEED_KEY = "Intake Speed";

  private final double HOLDER_SPEED_DEFAULT = 0.85;
  private double HOLDER_SPEED = HOLDER_SPEED_DEFAULT;
  private final String HOLDER_SPEED_KEY = "Holder Speed";

  public Intake(Joystick joystick_, XboxController controller_, LimitSwitch catapultLimitSwitch_, TimedLimitSwitch intakeLimitSwitch_) {
    joystick = joystick_;
    controller = controller_;
    catapultLimitSwitch = catapultLimitSwitch_;
    intakeLimitSwitch = intakeLimitSwitch_;

    intakeMotor = new CANSparkMax(Constants.DeviceIDs.INTAKE_MOTOR_ID,MotorType.kBrushless);
    //intakeMotor.setInverted(false);

    //intakSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.DeviceIDs.INTAKE_SOLENOID);

    hub = new PneumaticHub(Constants.DeviceIDs.PNEUMATIC_HUB);

    intakeSols = hub.makeDoubleSolenoid(Constants.DeviceIDs.INTAKE_SOLENOID1, Constants.DeviceIDs.INTAKE_SOLENOID2);

    dashboard = new SmartDashboardWrapper(this);
    dashboard.putNumber(INTAKE_SPEED_KEY, INTAKE_SPEED);
    dashboard.putNumber(HOLDER_SPEED_KEY, HOLDER_SPEED);
  }

  public void runIntake(){
    if (!catapultLimitSwitch.shouldIgnoreLimitSwitch() && !catapultLimitSwitch.get()) {
      // If the limit switch is not pressed (i.e. the Catapult is not down), then don't allow intake.
      return;
    }
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

  public void extendIntake() {
    intakeSols.set(Value.kForward);
  }

  public void retractIntake() {
    if (!catapultLimitSwitch.shouldIgnoreLimitSwitch() && !catapultLimitSwitch.get()) {
      // If the limit switch is not pressed (i.e. the Catapult is not down), then don't allow the Intake mechanism to be Retracted.
      return;
    }
    intakeSols.set(Value.kReverse);
  }

  public void intakeNominalSpeed() {
    intakeMotor.set(NOMINAL_SPEED);
  }

  private static final double CONTROLLER_SHOULDER_TRIGGER_THRESHOLD = 0.30;

  public boolean isIntakeButtonPressed()
  {
    if (Constants.twoDriverMode)
    {
      // Intake is the Right Trigger, which is an analog axis. It starts at 50%, so we'll call anything >= 70% as "held"
      return controller.getRightTriggerAxis() >= CONTROLLER_SHOULDER_TRIGGER_THRESHOLD || 
             controller.getRawButton(Constants.ControllerButtons.RIGHT_TRIGGER);
    }
    else
    {
      return joystick.getRawButton(Constants.JoyStickButtons.INTAKE_RUN);
    }
  }

  public boolean isIntakeReverseButtonPressed()
  {
    if (Constants.twoDriverMode)
    {
      // Intake Reverse (outtake?) is Left Trigger, which is an analog axis. It starts at 50%, so we'll call anything >= 70% as "held"
      return controller.getLeftTriggerAxis() >= CONTROLLER_SHOULDER_TRIGGER_THRESHOLD ||
             controller.getRawButton(Constants.ControllerButtons.LEFT_TRIGGER);
    }
    else
    {
      return joystick.getRawButton(Constants.JoyStickButtons.INTAKE_REVERSE);
    }
  }

  public boolean isExtendIntakeButtonPressed()
  {
    if (Constants.twoDriverMode)
    {
      
      return controller.getBButton();
    }
    else
    {
      return joystick.getRawButton(Constants.JoyStickButtons.INTAKE_FORWARD);
    }
  }

  public boolean isRetractIntakeButtonPressed()
  {
    if (Constants.twoDriverMode)
    {
      return controller.getXButton();
    }
    else
    {
      return joystick.getRawButton(Constants.JoyStickButtons.INTAKE_BACK);
    }
  }

  @Override
  public void periodic() {
    intakeLimitSwitch.update();

    INTAKE_SPEED = dashboard.getNumber(INTAKE_SPEED_KEY, INTAKE_SPEED_DEFAULT);
    HOLDER_SPEED = dashboard.getNumber(HOLDER_SPEED_KEY, HOLDER_SPEED_DEFAULT);
    // This method will be called once per scheduler run
    if(isIntakeButtonPressed()) {
      runIntake();
      //runHolder();
      //System.out.println("intaking balls");
    }else if(isIntakeReverseButtonPressed()) {
      runIntakeReverse();
      //runHolderReverse();
      //System.out.println("intake back");
    } else {
      stopIntake();
      //stopHolder();
    }
    
    if (isExtendIntakeButtonPressed()) {
      extendIntake();
      //System.out.println("intake sols forward");
    } else if (isRetractIntakeButtonPressed()) {
      retractIntake();
      //System.out.println("intake sols back"); 0
      
    }
  }
}
