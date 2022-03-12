// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.catalog.Catalog;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import java.util.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimitSwitch;
import frc.robot.utils.Constants.AutoMode;
import frc.robot.utils.Constants.JoyStickButtons;
import frc.robot.utils.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Chassis chassis;
  Intake intake;
  Catapult catapult;
  Climber climber;
  Joystick joystick;
  XboxController controller;
  DigitalInput limitSwitch_device;
  LimitSwitch limitSwitch;
  long autoInitTime;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    joystick = new Joystick(Constants.DeviceIDs.JOYSTICK_PORT);
    controller = new XboxController(Constants.DeviceIDs.CONTROLLER_PORT);
    limitSwitch_device = new DigitalInput(Constants.DeviceIDs.LIMIT_SWITCH);
    limitSwitch = new LimitSwitch(joystick, limitSwitch_device);
    // Instantiate robot parts with shared classes
    chassis = new Chassis(joystick, controller);
    intake = new Intake(joystick, controller, limitSwitch);
    catapult = new Catapult(joystick, controller, limitSwitch);
    climber = new Climber(joystick, controller);
    configureButtonBindings();
    autoInitTime = 0;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */

  public void teleopPeriodic()
  {
    chassis.periodic();
    intake.periodic();
    catapult.periodic();
    climber.periodic();
  }

  

  public void autonomousPeriodic()
  {
    final int SHOOT_MS = 2000;
    final int TAXI_MS = 2000;
    final double TAXI_SPEED = 0.2;

    if (autoInitTime == 0) {
      autoInitTime = System.currentTimeMillis();
    }
    long now = System.currentTimeMillis();
    long elapsed = now - autoInitTime;

    switch (Constants.autoMode)
    {
      case AutoMode.TAXI_ONLY: {
        if (elapsed < TAXI_MS) {
          chassis.autoDrive(TAXI_SPEED);
        } else {
          chassis.stop();
        }
        break;
      }

      case AutoMode.SHOOT_ONLY: {
        boolean fire = elapsed < SHOOT_MS;
        catapult.autoPeriodic(fire);
        break;
      }

      case AutoMode.SHOOT_THEN_TAXI: {
        boolean fire = elapsed < SHOOT_MS;
        catapult.autoPeriodic(fire);
        if (elapsed >= SHOOT_MS && elapsed < (SHOOT_MS+TAXI_MS)) {
          chassis.autoDrive(TAXI_SPEED);
        } else {
          chassis.stop();
        }
        break;
      }

      case AutoMode.TAXI_INTAKE_SHOOT: {
        
        break;
      }

      default: {
        break;
      }
    }
  }


  public void resetEncoders()
  {
    chassis.resetEncoder();
  }

  public void paintDashboard()
  {
    chassis.paintDashboard();
    catapult.paintDashboard();
  }
}
