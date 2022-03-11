// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.xml.catalog.Catalog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimitSwitch;
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

  public void resetEncoders()
  {
    chassis.resetEncoder();
  }

  public Command getAutonomousCommand() {

    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.RobotValues.ksVolts, Constants.RobotValues.kvVoltSecondsPerMeter, Constants.RobotValues.kaVoltSecondsSquaredPerMeter), Constants.RobotValues.kDriveKinematics, 10);

    TrajectoryConfig config = 
      new TrajectoryConfig(Constants.RobotValues.kMaxSpeedMetersPerSecond, Constants.RobotValues.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.RobotValues.kDriveKinematics).addConstraint(autoVoltageConstraint);

    Trajectory trajectory = 
      TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0)), List.of(new Translation2d(1,0)), new Pose2d(2,0, new Rotation2d(0)), config);

    RamseteCommand ramseteCommand = 
      new RamseteCommand(
        trajectory, 
        chassis::getPose2d, 
        new RamseteController(Constants.RobotValues.kRamseteB, Constants.RobotValues.kRamseteZeta), 
        new SimpleMotorFeedforward(Constants.RobotValues.ksVolts, Constants.RobotValues.kvVoltSecondsPerMeter, Constants.RobotValues.kaVoltSecondsSquaredPerMeter), 
        Constants.RobotValues.kDriveKinematics, 
        chassis::getWheelSpeeds, 
        new PIDController(Constants.RobotValues.kPDriveVel, 0, 0), 
        new PIDController(Constants.RobotValues.kPDriveVel, 0, 0), 
        chassis::tankDriveVolts,
        chassis);

    chassis.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> chassis.tankDriveVolts(0, 0));
  }

  public void paintDashboard()
  {
    chassis.paintDashboard();
    catapult.paintDashboard();
  }
}
