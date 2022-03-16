// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.ArrayList;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import java.util.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.dashboard.SmartDashboardWrapper;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimitSwitch;
import frc.robot.utils.Constants.AutoMode;
import frc.robot.utils.Constants.JoyStickButtons;
import frc.robot.utils.Constants.TwoBallAutoState;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotUtils.AutonStraightDrive;
import frc.robot.utils.RobotUtils.ComboBoxItem;

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
  AutonStraightDrive autonStraightDrive;
  SmartDashboardWrapper dashboard;
  
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
    autonStraightDrive = new AutonStraightDrive(chassis.getDiffDrive(), chassis.getNavX(), chassis.getDriveTrainEncoder());
    configureButtonBindings();
    autoInitTime = 0;
    
    dashboard = new SmartDashboardWrapper(this);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  public void robotInit()
  {
    ArrayList<ComboBoxItem> autoList = new ArrayList<ComboBoxItem>();
    autoList.add(new ComboBoxItem("Full Two-ball Auto", TwoBallAutoState.UNSPECIFIED));
    autoList.add(new ComboBoxItem("Pick up Second Ball and stop", TwoBallAutoState.AFTER_INTAKE_SECOND_CARGO));
    autoList.add(new ComboBoxItem("Shoot First Ball and dont move", TwoBallAutoState.AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN));
    dashboard.createAutoPicker(autoList);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */

  public void teleopInit()
  {
    resetEncoders();
  }

  public void teleopPeriodic()
  {
    chassis.periodic();
    intake.periodic();
    catapult.periodic();
    climber.periodic();
  }

  boolean autoDone = false;
  long time_intake_extended = 0;
  long time_first_shot_taken = 0;
  long time_intake_began_after_arrival = 0;
  long time_second_shot_taken = 0;
  long time_first_taxi_begin = 0;
  long time_return_taxi_begin = 0;
  long time_second_taxi_begin = 0;

  final double TAXI_DISTANCE = 1.5; // in meters (hopefully)
  final double MAX_TAXI_SPEED = 0.5;
  final int INTAKE_EXTEND_WAIT = 1500;
  final int FIRE_SHOT_WAIT = 1500;
  final int INTAKE_CARGO_WAIT = 1000;
  final double RETURN_DISTANCE = -TAXI_DISTANCE;
  final double SECOND_TAXI_DISTANCE = 1.25;

  final long TAXI_TIME_HARD_STOP = 5000;

  public void autonomousInit()
  {
    //TWO_BALL_STATE = TwoBallAutoState.AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN;
    TWO_BALL_STATE = TwoBallAutoState.INITIAL;
    autoDone = false;
    time_intake_extended = 0;
    time_first_shot_taken = 0;
    time_intake_began_after_arrival = 0;
    time_second_shot_taken = 0;
    time_first_taxi_begin = 0;
    time_return_taxi_begin = 0;
    time_second_taxi_begin = 0;
    resetEncoders();
  }

  int TWO_BALL_STATE = TwoBallAutoState.INITIAL;
  public void two_ball_auto()
  {
    chassis.trans_high_with_brake();
    SmartDashboard.putNumber("AUTO_STATE", TWO_BALL_STATE);

    /*
    if (dashboard.getAutoSelection() == TWO_BALL_STATE) {
      DriverStation.reportWarning("Hit Auto End Condition due to Dashboard selection of "+TWO_BALL_STATE, false);
      TWO_BALL_STATE = TwoBallAutoState.AUTO_DONE;
    }
    */

    switch (TWO_BALL_STATE) {
      case TwoBallAutoState.INITIAL: {
        intake.runIntake();
        intake.extendIntake();
        if (time_intake_extended == 0) {
          time_intake_extended = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - time_intake_extended;
        if (elapsed > INTAKE_EXTEND_WAIT) {
          TWO_BALL_STATE = TwoBallAutoState.BEFORE_FIRST_SHOT_INTAKE_EXTENDED;
          break;
        }
        break;
      }
      case TwoBallAutoState.BEFORE_FIRST_SHOT_INTAKE_EXTENDED: {
        intake.stopIntake();
        catapult.autoPeriodic(true);
        if (time_first_shot_taken == 0) {
          time_first_shot_taken = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - time_first_shot_taken;
        if (elapsed > FIRE_SHOT_WAIT) {
          TWO_BALL_STATE = TwoBallAutoState.AFTER_FIRST_SHOT_BEFORE_CATAPULT_DOWN;
        }
        break;
      }
      case TwoBallAutoState.AFTER_FIRST_SHOT_BEFORE_CATAPULT_DOWN: {
        catapult.autoPeriodic(false);
        if (limitSwitch.isPhysicalSwitchPressed()) {
          TWO_BALL_STATE = TwoBallAutoState.AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN;
          break;
        }
        break;
      }
      case TwoBallAutoState.AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN: {
        if (time_first_taxi_begin == 0) {
          time_first_taxi_begin = System.currentTimeMillis();
          resetEncoders();
        }
        // Ensure the robot doesn't go crazy. Stop after N seconds.
        long now = System.currentTimeMillis();
        long taxi_elapsed = now - time_first_taxi_begin;
        if (taxi_elapsed > TAXI_TIME_HARD_STOP) {
          DriverStation.reportError("Hit TAXI_TIME_HARD_STOP in AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN, elapsed="+taxi_elapsed+", time_first_taxi_begin="+time_first_taxi_begin+", now="+now, false);
          TWO_BALL_STATE = TwoBallAutoState.AUTO_DONE;
          break;
        }

        intake.runIntake();
        
        boolean arrived = autonStraightDrive.periodic(TAXI_DISTANCE, MAX_TAXI_SPEED);
        if (arrived) {
          if (time_intake_began_after_arrival == 0) {
            time_intake_began_after_arrival = System.currentTimeMillis();
          }
          long elapsed = System.currentTimeMillis() - time_intake_began_after_arrival;
          if (elapsed >= INTAKE_CARGO_WAIT) {
            TWO_BALL_STATE = TwoBallAutoState.AFTER_INTAKE_SECOND_CARGO;
            chassis.resetEncoders(); // To prepare for next journey.
            break;
          }
        }
        break;
      }
      case TwoBallAutoState.AFTER_INTAKE_SECOND_CARGO: {
        intake.runIntake(); // Keep running the intake - we don't want to lose it,
                            // and we might not even fully have it in the claw yet!

        if (time_return_taxi_begin == 0) {
          time_return_taxi_begin = System.currentTimeMillis();
        }
        // Ensure the robot doesn't go crazy. Hard stop after N seconds.
        long now = System.currentTimeMillis();
        long taxi_elapsed = now - time_return_taxi_begin;
        if (taxi_elapsed > TAXI_TIME_HARD_STOP) {
          DriverStation.reportError("Hit TAXI_TIME_HARD_STOP in AFTER_ITNAKE_SECOND_CARGO, elapsed="+taxi_elapsed+", time_return_taxi_begin="+time_return_taxi_begin+", now="+now, false);
          TWO_BALL_STATE = TwoBallAutoState.AUTO_DONE;
          break;
        }

        boolean arrived = autonStraightDrive.periodic(RETURN_DISTANCE, MAX_TAXI_SPEED);
        if (arrived) {
          TWO_BALL_STATE = TwoBallAutoState.AFTER_RETURN_TO_ORIGINAL_SPOT;
          chassis.resetEncoders(); // To prepare for next journey.
          break;
        }
        break;
      }
      case TwoBallAutoState.AFTER_RETURN_TO_ORIGINAL_SPOT: {
        // At this point we can finally stop the intake.
        intake.stopIntake();
        catapult.autoPeriodic(true);
        if (time_second_shot_taken == 0) {
          time_second_shot_taken = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - time_second_shot_taken;
        if (elapsed > FIRE_SHOT_WAIT) {
          TWO_BALL_STATE = TwoBallAutoState.AFTER_SECOND_SHOT;
          break;
        }
        break;
      }
      case TwoBallAutoState.AFTER_SECOND_SHOT: {
        if (time_second_taxi_begin == 0) {
          time_second_taxi_begin = System.currentTimeMillis();
        }
        long now = System.currentTimeMillis();
        long taxi_elapsed = now - time_second_taxi_begin;
        // Ensure the robot doesn't go crazy. Hard stop after N seconds.
        if (taxi_elapsed > TAXI_TIME_HARD_STOP) {
          DriverStation.reportError("Hit TAXI_TIME_HARD_STOP in AFTER_SECOND_SHOT, elapsed="+taxi_elapsed+", time_second_taxi_begin="+time_second_taxi_begin+", now="+now, false);
          TWO_BALL_STATE = TwoBallAutoState.AUTO_DONE;
          break;
        }

        catapult.autoPeriodic(false);
        boolean arrived = autonStraightDrive.periodic(SECOND_TAXI_DISTANCE, MAX_TAXI_SPEED);
        if (arrived) {
          TWO_BALL_STATE = TwoBallAutoState.AUTO_DONE;
          break;
        }
        break;
      }
      
      case TwoBallAutoState.AUTO_DONE:
      default: {
        // Continue on this until it's time for Teleop!
        catapult.autoPeriodic(false);
        chassis.stop();
        intake.stopIntake();

        break;
      }
    }

  }

  public void autonomousPeriodic()
  {
    chassis.runCompressor();
    if (Constants.autoMode == AutoMode.TWO_BALL_AUTO) {
      two_ball_auto();
      return;
    }
/*

    chassis.low();
    if (autoDone) {
      chassis.stop();
      return;
    }
    switch (Constants.autoMode) {
      case AutoMode.TAXI_ONLY: {
        if (autonStraightDrive.periodic(TAXI_DISTANCE, MAX_TAXI_SPEED)) {
          autoDone = true;
          chassis.stop();
        }
        break;
      }
    }
    */
  }

  public void autonomousTimedPeriodic()
  {
    final int SHOOT_MS = 2000;
    final int TAXI_MS = 2000;
    final int INTAKE_EXTEND_MS = 1000;
    final double TAXI_SPEED = 0.2;

    if (autoInitTime == 0) {
      autoInitTime = System.currentTimeMillis();
    }
    long now = System.currentTimeMillis();
    long elapsed = now - autoInitTime;

    // no matter what, we always want to extend the intake immediately & always.
    intake.extendIntake();

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
        if (elapsed > INTAKE_EXTEND_MS) {
          boolean fire = elapsed < SHOOT_MS+INTAKE_EXTEND_MS;
          catapult.autoPeriodic(fire);
        }
        break;
      }

      case AutoMode.SHOOT_THEN_TAXI: {
        if (elapsed > INTAKE_EXTEND_MS) {
          boolean fire = elapsed < SHOOT_MS+INTAKE_EXTEND_MS;
          catapult.autoPeriodic(fire);
          if (elapsed >= SHOOT_MS+INTAKE_EXTEND_MS && elapsed < (SHOOT_MS+INTAKE_EXTEND_MS+TAXI_MS)) {
            chassis.autoDrive(TAXI_SPEED);
          } else {
            chassis.stop();
          }
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
    autoDone = false;
    chassis.resetEncoders();
  }
  /*
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
  */

  public void paintDashboard()
  {
    chassis.paintDashboard();
    catapult.paintDashboard();
  }
}
