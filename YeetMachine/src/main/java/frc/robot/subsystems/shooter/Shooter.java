// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax leftNeo = new CANSparkMax(ShooterConstants.leftNeo,MotorType.kBrushless);
  CANSparkMax rightNeo= new CANSparkMax(ShooterConstants.rightNeo,MotorType.kBrushless);
  Joystick joystick = new Joystick(0);
  double power = 0;

  public Shooter() {

  }

  public void actShooter(){
    if(joystick.getTrigger()){
      leftNeo.set(ShooterConstants.shooterPower );
      rightNeo.set(-ShooterConstants.shooterPower);
      for(double i = 0; i != ShooterConstants.shooterPower; i+=.005){
        power = i;
      }
      leftNeo.set(power);
      rightNeo.set(-power);
    }else{
      leftNeo.set(0);
      rightNeo.set(0);
    }
  }
  
  public void eject(){

  }

  public void lightEject(){
   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    actShooter();
  }
}
