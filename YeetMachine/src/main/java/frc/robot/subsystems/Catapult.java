// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.chrono.MinguoEra;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.SmartDashboardWrapper;
import frc.robot.utils.Constants;

public class Catapult extends SubsystemBase {

  /** Creates a new Catapult. */
  CANSparkMax yeeter;
  Joystick joystick;
  XboxController controller;
  SmartDashboardWrapper dashboard;
  LimitSwitch limitSwitch;
  
  private final double YEETER_SPEED_DEFAULT = 1.0;
  private double YEETER_SPEED = YEETER_SPEED_DEFAULT;
  private final String YEETER_SPEED_KEY = "Yeeter Speed";

  NetworkTable table;;
  NetworkTableEntry tx, ty, ta;
  double x, y, area;

  public Catapult(Joystick joystick_, XboxController controller_, LimitSwitch limitSwitch_) {
    limitSwitch = limitSwitch_;
    joystick = joystick_;
    controller = controller_;
    yeeter = new CANSparkMax(Constants.DeviceIDs.CATAPULT_ID,MotorType.kBrushless);
    dashboard = new SmartDashboardWrapper(this);
    yeeter.setIdleMode(IdleMode.kBrake);
    dashboard.putNumber(YEETER_SPEED_KEY, YEETER_SPEED);

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  public void pewpew(){
    final double invertedSpeed = -1 * Math.abs(YEETER_SPEED);
    yeeter.set(invertedSpeed);
  }

  public void nopewpew(){
    yeeter.set(0.0);
  }
  
  public void paintDashboard() {
    dashboard.putBoolean("Yeeter Down", limitSwitch.isPhysicalSwitchPressed());
    dashboard.putBoolean("Manual Mode", limitSwitch.shouldIgnoreLimitSwitch());
  }
  
  public boolean isFireButtonPressed() {
    //if (Constants.twoDriverMode
    return joystick.getRawButton(Constants.JoyStickButtons.CATAPULT);
  }

  public boolean limeLightOnTarget(){

    boolean check1 = false, check2 = false, check3 = false;

    if(x < 0 && x > 0){
      check1 = true;
    }
    if(y < 0 && y > 0){
      check2 = true;
    }
    if(area < 0 && area > 0){
      check3 = true;
    }

    return (check1 == check2) == check3;
  }

  public void autoPeriodic(boolean fire)
  {
    if (limitSwitch.shouldIgnoreLimitSwitch()) {
      if (fire) {
        pewpew();
      } else {
        nopewpew();
      }
    } else {
      if (limitSwitch.get(fire)) {
        nopewpew();
      } else {
        pewpew();
      }
    }
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run

    /** Catapult Logic:
     * The motor will always move as long as the limit switch reads false
     * If the switch reads true, the motor will stop, making that the "home position"
     * If the trigger is pressed while at the home positin, take a shoot
     * Rinse, Repeat until Einstien
     */

    YEETER_SPEED = dashboard.getNumber(YEETER_SPEED_KEY, YEETER_SPEED_DEFAULT);

    /* "Manual" behavior */
    if (limitSwitch.shouldIgnoreLimitSwitch())
    {
      if(isFireButtonPressed()) {
        pewpew();
      }else{
        nopewpew();
      }
    } else {
      if(limitSwitch.get(isFireButtonPressed())) {
        nopewpew();
      }else {
        pewpew();
      }
    }

    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
        
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
} 