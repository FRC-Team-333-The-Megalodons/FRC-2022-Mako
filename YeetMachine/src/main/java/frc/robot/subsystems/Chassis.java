// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.element.Element;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */

  //CANSPARKS
  CANSparkMax leftLeader, leftFollower, leftFollower2;
  CANSparkMax rightLeader, rightFollower, rightFollower2;

  //ENCODERS
  RelativeEncoder leftLeaderEnc, leftFollowerEnc, leftFollower2Enc;
  RelativeEncoder rightLeaderEnc, rightFollowerEnc, rightFollower2Enc;

  //dif drive
  DifferentialDrive differentialDrive;

  //JoyStick delcaration
  Joystick joystick;

  //transmission and compressor
  Compressor compressor;
  DoubleSolenoid solenoids;

  public Chassis() {
    
    //motor controller declarations
    leftLeader = new CANSparkMax(Constants.DeviceIDs.LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new CANSparkMax(Constants.DeviceIDs.LEFT_FOLLOWER_ID, MotorType.kBrushless);
    leftFollower.follow(leftLeader);
    //leftFollower2 = new CANSparkMax(Constants.DeviceIDs.LEFT_FOLLOWER2_ID, MotorType.kBrushless);

    rightLeader = new CANSparkMax(Constants.DeviceIDs.RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new CANSparkMax(Constants.DeviceIDs.RIGHT_FOLLOWER_ID, MotorType.kBrushless);
    rightFollower.follow(rightLeader);
    //rightFollower2 = new CANSparkMax(Constants.DeviceIDs.LEFT_FOLLOWER2_ID, MotorType.kBrushless);

    //motor controller encoder declarations
    leftLeaderEnc = leftLeader.getEncoder();
    leftFollowerEnc = leftFollower.getEncoder();
    //leftFollower2Enc = leftFollower2.getEncoder();

    rightLeaderEnc = rightLeader.getEncoder();
    rightFollowerEnc = rightFollower.getEncoder();
    //rightFollower2Enc = rightFollower2.getEncoder();

    //setting inverts
    leftLeader.setInverted(false);
    leftFollower.setInverted(false);
    //leftFollower2.setInverted(false);

    rightLeader.setInverted(false);
    rightFollower.setInverted(false);
    //rightFollower2.setInverted(false);

    //dif drive delcaration
    differentialDrive = new DifferentialDrive(leftLeader,rightLeader);

    joystick = new Joystick(Constants.DeviceIDs.JOYSTICK_PORT);

    //compressor = new Compressor(PneumaticsModuleType.REVPH);

    //solenoids = new DoubleSolenoid(0,PneumaticsModuleType.REVPH,0,1);//todo change solenoid number
  }

  public void manualDrive(){
    /*if(joystick.getRawButton(5)){
      solenoids.set(Value.kForward);
    }
    if(joystick.getRawButton(6)){
      solenoids.set(Value.kReverse);   TODO: joey commented this out
    }*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    differentialDrive.arcadeDrive(joystick.getX(), -joystick.getY());
    //compressor.enableDigital();
  }
}
