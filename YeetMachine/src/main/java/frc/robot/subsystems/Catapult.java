// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.Constants;

public class Catapult extends SubsystemBase {
  /** Creates a new Catapult. */

  PneumaticHub hub;
  Solenoid catapult;
  Joystick joystick;

  public Catapult() {
    hub = new PneumaticHub(Constants.DeviceIDs.HUB_PORT);
    catapult = hub.makeSolenoid(2);
    joystick = new Joystick(0);
  }

  public void pewpew(){
    catapult.set(true);
  }

  public void nopewpew(){
    catapult.set(false);
  }

  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    if(joystick.getRawButton(Constants.JoyStickButtons.CATAPULT)){//TODO set constants
      pewpew();
      //System.out.println("pew pew");
    }
    if(joystick.getRawButton(Constants.JoyStickButtons.CATAPULT_DOWN)){
      nopewpew();
      //System.out.println("no pew pew");
    }
   }
}

