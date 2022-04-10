// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.dashboard.SmartDashboardWrapper;
import frc.robot.subsystems.LimitSwitch.TimedLimitSwitch;
import frc.robot.utils.Constants;

public class Catapult extends SubsystemBase {

  /** Creates a new Catapult. */
  CANSparkMax yeeter;
  Joystick joystick;
  XboxController controller;
  SmartDashboardWrapper dashboard;
  LimitSwitch catapultLimitSwitch;
  TimedLimitSwitch intakeLimitSwitch;
  
  private final double YEETER_SPEED_DEFAULT = 0.5;
  private double YEETER_SPEED = YEETER_SPEED_DEFAULT;
  private final String YEETER_SPEED_KEY = "Yeeter Speed";

  NetworkTable table;
  NetworkTableEntry tx, ty, ta;
  double x, y, area;

  public Catapult(Joystick joystick_, XboxController controller_, LimitSwitch catapultLimitSwitch_, TimedLimitSwitch intakeLimitSwitch_) {
    catapultLimitSwitch = catapultLimitSwitch_;
    intakeLimitSwitch = intakeLimitSwitch_;
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
    dashboard.putBoolean("Yeeter Down", catapultLimitSwitch.get());
    dashboard.putBoolean("YeeterDown RAW ", catapultLimitSwitch.isPhysicalSwitchPressed());
    dashboard.putBoolean("Manual Mode", catapultLimitSwitch.shouldIgnoreLimitSwitch());
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

  public void catapultPeriodic(boolean fire)
  {
    /* MANUAL OVERRIDE MODE:
       Ignore all limit switches, and ONLY run when fire true */
    if (catapultLimitSwitch.shouldIgnoreLimitSwitch()) {
      if (fire) {
        pewpew();
      } else {
        nopewpew();
      }
      return;
    }

    /* REGULAR OPERATING MODE (BOTH PERIODIC AND AUTONOMOUS):
       Observe limit switches, and always try to bring the catapult to CATAPULT LIMIT SWITCH PRESSED State */


    // If the Intake is in , then force fire to 'false' no matter what (It's not safe to fire, but it is safe to bring the catapult down)
    if (intakeLimitSwitch.get()) {
      fire = false;
    }

    if (catapultLimitSwitch.get(fire)) {
      // If the catapult limitswitch is pressed, don't run (unless fire is true)
      nopewpew();
    } else {
      // If the catapult limitswitch is not pressed, run until it's pressed.
      pewpew();
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
     * 
     * Potential High Goal Logic:
     * if the limelight dist_to_target is within the distance needed to take the high goal, 
     * change the shot bool on the dashboard to true 
     */

    YEETER_SPEED = dashboard.getNumber(YEETER_SPEED_KEY, YEETER_SPEED_DEFAULT);

    catapultPeriodic(isFireButtonPressed());

    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
        
    /*
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    */
  }
} 