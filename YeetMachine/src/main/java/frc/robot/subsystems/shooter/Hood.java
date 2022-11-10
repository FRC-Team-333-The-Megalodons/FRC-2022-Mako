// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */

  CANSparkMax hoodNeo;
  RelativeEncoder encoder;
  Joystick stick = new Joystick(0);

  public Hood() {
    hoodNeo = new CANSparkMax(ShooterConstants.HoodConstants.hoodNeo,MotorType.kBrushless);
    hoodNeo.setIdleMode(IdleMode.kBrake);
    encoder = hoodNeo.getEncoder();
    encoder.setInverted(true);
    //We still have no fixed design for this, however we will use one neo motor, Specifically the NEO 550. It is a brushless motor that will use a sparkMAX
    // i will take care of PID stuff, you define the class
  }

  //****************IGNORE THE METHODS BELOW EXCEPT PERIODIC(), THEY ARE FOR THE FUTURE****************

  private double error = 0, errorSum = 0, errorRate, lastError = 0, lastTimeStamp = 0, currentTime = 0;

  public void resetEncoder(){
    encoder.setPosition(0);
    errorSum = 0;
    lastError = 0;
    lastTimeStamp = Timer.getFPGATimestamp();
  }

  public void setFenderShot(){

  }

  public void setTaxiLineShot(){

  }

  public void setLaunchPadShot(){
    currentTime = Timer.getFPGATimestamp() - lastTimeStamp;
    if(Math.abs(error) < ShooterConstants.HoodConstants.kILIM){
      errorSum += error + currentTime;
    }

    errorRate = (error - lastError)/currentTime;

    //error = Constants.HoodConstants.HOME - encoder.getPosition();
    
    hoodNeo.set(getOutput(error, errorSum, errorRate));
    //update time stamp & error
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = error;
  }

  public void setHome(){
    currentTime = Timer.getFPGATimestamp() - lastTimeStamp;
    if(Math.abs(error) < ShooterConstants.HoodConstants.kILIM && Math.abs(error) > 2){
      errorSum += error + currentTime;
    }

    errorRate = (error - lastError)/currentTime;

    //error = Constants.HoodConstants.HOME - encoder.getPosition();
    
    hoodNeo.set(getOutput(error, errorSum, errorRate));
    //update time stamp & error
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = error;
  }

  public void manualAdjust(){
    hoodNeo.set(stick.getRawAxis(1));
  }

  public void update(double target){
    error = target - encoder.getPosition();
  }

  private double getOutput(double error, double errorSum, double errorRate){
    return (ShooterConstants.HoodConstants.kP * error) + (ShooterConstants.HoodConstants.kI * errorSum) + (ShooterConstants.HoodConstants.kD * errorRate);
  }

  public void displayVals(){
    SmartDashboard.putNumber("Hood position", encoder.getPosition());
  }

  boolean setHome = false;
  double target = 0;

  public void init(){
    resetEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displayVals();
    manualAdjust();
    System.out.println(error);
    if(stick.getTrigger()){
      setHome = true;
      target = ShooterConstants.HoodConstants.HOME;
    }
    if(stick.getRawButton(2)){
      setHome = false;
      target = ShooterConstants.HoodConstants.LAUNCHPAD;
    }

    update(target);

    if(setHome){
      setHome();
    }else{
      setLaunchPadShot();
    }
  }
}
