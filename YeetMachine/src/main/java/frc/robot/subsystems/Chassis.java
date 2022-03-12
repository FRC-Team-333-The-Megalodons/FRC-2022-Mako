// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import javax.lang.model.element.Element;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.SmartDashboardWrapper;
import frc.robot.utils.Constants;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */

  //CANSPARKS
  CANSparkMax leftLeader, leftFollower, leftFollower2;
  CANSparkMax rightLeader, rightFollower, rightFollower2;

  ArrayList<CANSparkMax> allDrivetrainSpeedcontrollers;

  //ENCODERS
  RelativeEncoder leftLeaderEnc, leftFollowerEnc, leftFollower2Enc;
  RelativeEncoder rightLeaderEnc, rightFollowerEnc, rightFollower2Enc;

  //dif drive
  DifferentialDrive differentialDrive;

  //JoyStick delcaration
  Joystick joystick;
  XboxController controller;

  //transmission and compressor
  PneumaticHub hub = new PneumaticHub(Constants.DeviceIDs.PNEMATIC_HUB);
  DoubleSolenoid solenoids;

  AHRS navx;

  DifferentialDriveOdometry odometry;

  SmartDashboardWrapper dashboard;

  public Chassis(Joystick joystick_, XboxController controller_) {
    joystick = joystick_;
    controller = controller_;
    dashboard = new SmartDashboardWrapper(this);
    //motor controller declarations
    leftLeader = new CANSparkMax(Constants.DeviceIDs.LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new CANSparkMax(Constants.DeviceIDs.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    leftFollower.follow(leftLeader);
    leftFollower2 = new CANSparkMax(Constants.DeviceIDs.LEFT_FOLLOWER2_ID, MotorType.kBrushless);
    leftFollower2.follow(leftLeader);

    rightLeader = new CANSparkMax(Constants.DeviceIDs.RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new CANSparkMax(Constants.DeviceIDs.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
    rightFollower.follow(rightLeader);
    rightFollower2 = new CANSparkMax(Constants.DeviceIDs.RIGHT_FOLLOWER2_ID, MotorType.kBrushless);
    rightFollower2.follow(rightLeader);

    allDrivetrainSpeedcontrollers = new ArrayList<CANSparkMax>();
    allDrivetrainSpeedcontrollers.add(leftLeader);
    allDrivetrainSpeedcontrollers.add(leftFollower);
    allDrivetrainSpeedcontrollers.add(leftFollower2);
    allDrivetrainSpeedcontrollers.add(rightLeader);
    allDrivetrainSpeedcontrollers.add(rightFollower);
    allDrivetrainSpeedcontrollers.add(rightFollower2);

    //motor controller encoder declarations
    leftLeaderEnc = leftLeader.getEncoder();
    leftFollowerEnc = leftFollower.getEncoder();
    leftFollower2Enc = leftFollower2.getEncoder();

    rightLeaderEnc = rightLeader.getEncoder();
    rightFollowerEnc = rightFollower.getEncoder();
    rightFollower2Enc = rightFollower2.getEncoder();

    leftLeaderEnc.setPositionConversionFactor(1/Constants.RobotValues.kHallEffectUnitsPerMeter);
    rightLeaderEnc.setPositionConversionFactor(1/Constants.RobotValues.kHallEffectUnitsPerMeter); // TODO: CHeck if we should use different numbers for left & right.
    
    //setting inverts

    for (CANSparkMax c : allDrivetrainSpeedcontrollers) {
      c.setInverted(false);
      c.setIdleMode(IdleMode.kCoast);
    }


    //dif drive delcaration
    differentialDrive = new DifferentialDrive(leftLeader,rightLeader);

    solenoids = hub.makeDoubleSolenoid(Constants.DeviceIDs.DRIVETRAIN_SOLENOID_LOW, Constants.DeviceIDs.DRIVETRAIN_SOLENOID_HIGH);

    navx = new AHRS(SPI.Port.kMXP);
  }

  public void low(){
    for (CANSparkMax c : allDrivetrainSpeedcontrollers) {
      c.setIdleMode(IdleMode.kBrake);
    }
    solenoids.set(Value.kReverse);
  }

  public void high(){
    for (CANSparkMax c : allDrivetrainSpeedcontrollers) {
      c.setIdleMode(IdleMode.kCoast);
    }
    solenoids.set(Value.kForward);
  }

  /*
  // For now, we don't need automatic transmission, as we can just always use high gear unless we're in defense.
  public void autoTrans(){
    if(joystick.getY() < .4){
      low();
    }else if(joystick.getY() >= .8){
      high();
    }
  }
  */

  public void resetEncoder()
  {
    leftLeaderEnc.setPosition(0);
    rightLeaderEnc.setPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftLeaderEnc.getVelocity(), rightLeaderEnc.getVelocity());
  }

  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public void paintDashboard()
  {
    dashboard.putNumber("LeftLeaderEnc", leftLeaderEnc.getPosition());
    dashboard.putNumber("RightLeaderEnc",rightLeaderEnc.getPosition());
  }

  public void autoDrive(double xSpeed)
  {
    double rotation = 0.0; // TODO : Get from NavX
    arcadeDrive(xSpeed, rotation);
  }

  public void arcadeDrive(double xSpeed, double zRotation)
  {
    differentialDrive.arcadeDrive(xSpeed, zRotation);
  }

  public void stop()
  {
    arcadeDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //odometry.update(navx.getRotation2d(), leftLeaderEnc.getPosition(), rightLeaderEnc.getPosition());
   arcadeDrive(joystick.getX(), -joystick.getY());//joystick.getX(), -joystick.getY()

    hub.enableCompressorAnalog(100, 110);

    if(joystick.getRawButton(Constants.JoyStickButtons.LOW_GEAR) ||
       joystick.getRawButton(Constants.JoyStickButtons.LOW_GEAR_EXTRA1) ||
       joystick.getRawButton(Constants.JoyStickButtons.LOW_GEAR_EXTRA2))
    {
      low();
    }
    else
    {
      high();
    }
  }
}
