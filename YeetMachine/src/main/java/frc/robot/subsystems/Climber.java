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
import frc.robot.dashboard.SmartDashboardWrapper;
import frc.robot.utils.Constants;

public class Climber extends SubsystemBase {

  CANSparkMax leftClimber;
  CANSparkMax rightClimber;
  Joystick joystick;
  XboxController controller;
  DoubleSolenoid doubleClimbers;
  PneumaticHub hub;
  SmartDashboardWrapper dashboard;
  LimitSwitch catapultLimitSwitch;

  private final double CLIMB_SPEED = 1.0;
  private final double LEFT_CLIMB_SPEED = -CLIMB_SPEED;
  private final double RIGHT_CLIMB_SPEED = CLIMB_SPEED;
  private final double LEFT_CLIMBER_LIMIT = -200.0;
  private final double RIGHT_CLIMBER_LIMIT = 200.0;
  //private final double LEFT_CLIMB_LOWER_LIMIT = -15.0;
  //private final double RIGHT_CLIMB_LOWER_LIMIT = 15.0;
  private boolean climberAutomaticMode = false;

  /** Creates a new Climber. */
  public Climber(Joystick joystick_, XboxController controller_, LimitSwitch catapultLimitSwitch_)
  {
    joystick = joystick_;
    controller = controller_;
    catapultLimitSwitch = catapultLimitSwitch_;

    leftClimber = new CANSparkMax(Constants.DeviceIDs.LEFT_CLIMBER_ID, MotorType.kBrushless);
    rightClimber = new CANSparkMax(Constants.DeviceIDs.RIGHT_CLIMBER_ID, MotorType.kBrushless);

    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    hub = new PneumaticHub(Constants.DeviceIDs.PNEUMATIC_HUB);

    doubleClimbers = hub.makeDoubleSolenoid(2,3);//todo change this

    dashboard = new SmartDashboardWrapper(this);
  }

  public boolean hasReachedLimit(double speed, double current, double limit)
  {
    if (speed >= 0) {
      // If we're "going up", then see if current >= limit.
      return current >= limit;
    } else {
      // If we're "going down", then see if current < limit.
      return current < limit;
    }
  }

  final double JOYSTICK_THRESHOLD = 0.7;

  public boolean isManualRightUpPressed()
  {
    return controller.getRightY() < -JOYSTICK_THRESHOLD;
  }

  public boolean isManualRightDownPressed()
  {
    return controller.getRightY() > JOYSTICK_THRESHOLD;
  }

  public boolean isManualLeftUpPressed()
  {
    return controller.getLeftY() < -JOYSTICK_THRESHOLD;
  }

  public boolean isManualLeftDownPressed()
  {
    return controller.getLeftY() > JOYSTICK_THRESHOLD;
  }

  public boolean isAnyManualPressed()
  {
    return isManualLeftDownPressed() ||
           isManualLeftUpPressed() ||
           isManualRightDownPressed() ||
           isManualRightUpPressed();
  }


  public void climbUp() {
    
    // Keep original values "up".
    double left_speed = LEFT_CLIMB_SPEED;
    double right_speed = RIGHT_CLIMB_SPEED; 
    if (climberAutomaticMode && hasReachedLimit(left_speed, leftClimber.getEncoder().getPosition(), LEFT_CLIMBER_LIMIT))
    {
      // We're in Automatic and we've hit the Limit. Stop.
      leftClimber.set(0);
    } else {
      leftClimber.set(left_speed);
    }
 
    if (climberAutomaticMode && hasReachedLimit(right_speed, rightClimber.getEncoder().getPosition(), RIGHT_CLIMBER_LIMIT))
    {
      rightClimber.set(0);
    } else {
      rightClimber.set(right_speed);
    }
  }

  public void climbDown() {
    // Invert both for "down".
    double left_speed = -1 * LEFT_CLIMB_SPEED;
    double right_speed = -1 * RIGHT_CLIMB_SPEED; 
    
    /*
    if (climberAutomaticMode && hasReachedLimit(left_speed, leftClimber.getEncoder().getPosition(), LEFT_CLIMB_LOWER_LIMIT))
    {
      leftClimber.set(0);
    } else {
      leftClimber.set(left_speed);
    }

    if (climberAutomaticMode && hasReachedLimit(right_speed, rightClimber.getEncoder().getPosition(), RIGHT_CLIMB_LOWER_LIMIT))
    {
      leftClimber.set(0);
    } else {
      rightClimber.set(right_speed);
    }
    */
    
    leftClimber.set(left_speed);
    rightClimber.set(right_speed);
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
    return controller.getRawButton(6);
  }

  public boolean isDoubleClimberDownButtonPressed(){
    return controller.getRawButton(5);
  }

  public void followManualControls()
  {
    // Left and right can be at the same time.
    if (isManualLeftUpPressed()) {
      leftClimber.set(LEFT_CLIMB_SPEED);
    } else if (isManualLeftDownPressed()) {
      leftClimber.set(-LEFT_CLIMB_SPEED);
    } else {
      leftClimber.set(0.0);
    }

    // No "else" between left & right, as they can be simultaneous/independent.

    if (isManualRightUpPressed()) {
      rightClimber.set(RIGHT_CLIMB_SPEED);
    } else if (isManualRightDownPressed()) {
      rightClimber.set(-RIGHT_CLIMB_SPEED);
    } else {
      rightClimber.set(0.0);
    }
  }


  @Override
  public void periodic() {
    climberAutomaticMode = !catapultLimitSwitch.shouldIgnoreLimitSwitch();
    // This method will be called once per scheduler run
    if(isRaiseClimberButtonPressed()) {
      climbUp();
    } else if (isLowerClimberButtonPressed()) {
      climbDown();
    } else if (isAnyManualPressed()) {
      followManualControls();
    } else {
      stopClimb();  
    }

    if(controller.getRightBumper()){
      doubleClimbersUp();
      //System.out.println("oui");
    }

    if(controller.getLeftBumper()){
      doubleClimbersDown();
      //System.out.println("non");
    }
  } 

  public void teleopInit()
  {
    // Zeroes the climber values when Periodic is enabled (TODO: Confirm this actually works.)
    leftClimber.getEncoder().setPosition(0);
    rightClimber.getEncoder().setPosition(0);
  }
  
  public void paintDashboard()
  {
    dashboard.putNumber("LeftClimber", leftClimber.getEncoder().getPosition());
    dashboard.putNumber("RightClimber", rightClimber.getEncoder().getPosition());
  }
}

