// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.dashboard.SmartDashboardWrapper;
import frc.robot.subsystems.Catapult;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimitSwitch;
import frc.robot.subsystems.LimitSwitch.TimedLimitSwitch;
import frc.robot.utils.Constants.AutoState;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotUtils.AutoOption;
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
  DigitalInput catapultLimitSwitch_raw, intakeLimitSwitch_raw;
  LimitSwitch catapultLimitSwitch;
  TimedLimitSwitch intakeLimitSwitch;
  long autoInitTime;
  AutonStraightDrive autonStraightDrive;
  SmartDashboardWrapper dashboard;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    joystick = new Joystick(Constants.DeviceIDs.JOYSTICK_PORT);
    controller = new XboxController(Constants.DeviceIDs.CONTROLLER_PORT);
    catapultLimitSwitch_raw = new DigitalInput(Constants.DeviceIDs.CATAPULT_LIMIT_SWITCH);
    catapultLimitSwitch = new LimitSwitch(joystick, catapultLimitSwitch_raw);
    intakeLimitSwitch_raw = new DigitalInput(Constants.DeviceIDs.INTAKE_LIMIT_SWITCH);
    intakeLimitSwitch = new TimedLimitSwitch(intakeLimitSwitch_raw);
    // Instantiate robot parts with shared classes
    chassis = new Chassis(joystick, controller);
    intake = new Intake(joystick, controller, catapultLimitSwitch, intakeLimitSwitch);
    catapult = new Catapult(joystick, controller, catapultLimitSwitch, intakeLimitSwitch);
    climber = new Climber(joystick, controller, catapultLimitSwitch);
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
    autoList.add(new ComboBoxItem("Do nothing", new AutoOption(AutoState.AUTO_DONE, AutoState.AUTO_DONE)));
    autoList.add(new ComboBoxItem("Shoot only", new AutoOption(AutoState.INITIAL, AutoState.AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN)));
    autoList.add(new ComboBoxItem("Taxi only", new AutoOption(AutoState.AFTER_FIRST_SHOT_BEFORE_CATAPULT_DOWN, AutoState.AFTER_INTAKE_SECOND_CARGO)));
    autoList.add(new ComboBoxItem("Shoot, Taxi & Intake", new AutoOption(AutoState.INITIAL, AutoState.AFTER_INTAKE_SECOND_CARGO)));
    autoList.add(new ComboBoxItem("Full Two-ball Auto", new AutoOption(AutoState.INITIAL, AutoState.AUTO_DONE)));
    dashboard.createAutoPicker(autoList);

    dashboard.putNumber("TAXI_DISTANCE", DEFAULT_TAXI_DISTANCE);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */

  public void teleopInit()
  {
    resetEncoders();
    climber.teleopInit();
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
  long time_began_to_bring_down_catapult = 0;

  final double DEFAULT_TAXI_DISTANCE = 1.9; // in meters (hopefully)
  double TAXI_DISTANCE = DEFAULT_TAXI_DISTANCE;
  final double MAX_TAXI_SPEED = 0.5;
  final int INTAKE_EXTEND_WAIT = 4000;
  final int FIRE_SHOT_WAIT = 1500;
  final int INTAKE_CARGO_WAIT = 1000;
  final int TIME_BRING_DOWN_CATAPULT_WAIT = 4000;
  double RETURN_DISTANCE = -TAXI_DISTANCE;
  final double SECOND_TAXI_DISTANCE = 1.5;
  final double ADDITIONAL_INTAKE_POWER = 0.1; // Compensates for hitting the ball slowly

  final long TAXI_TIME_HARD_STOP = 5000;
  int TWO_BALL_STATE = AutoState.UNSPECIFIED;
  int TWO_BALL_STOP_STATE = AutoState.UNSPECIFIED;
  boolean dontIntakeDuringAuto = false;

  public void autonomousInit()
  {
    DriverStation.reportWarning("AUTONOMOUS_INIT", false);
    TAXI_DISTANCE = dashboard.getNumber("TAXI_DISTANCE", DEFAULT_TAXI_DISTANCE);
    RETURN_DISTANCE = -TAXI_DISTANCE;
    DriverStation.reportWarning("Auto Taxi Distance = " + TAXI_DISTANCE, false);
    // THis begins at the "start state" from the selected dashboard option.
    TWO_BALL_STATE = dashboard.getAutoSelection().start_state;
    TWO_BALL_STOP_STATE = dashboard.getAutoSelection().stop_state;
    dontIntakeDuringAuto = false;
    autoDone = false;
    time_intake_extended = 0;
    time_first_shot_taken = 0;
    time_began_to_bring_down_catapult = 0;
    time_intake_began_after_arrival = 0;
    time_second_shot_taken = 0;
    time_first_taxi_begin = 0; 
    time_return_taxi_begin = 0;
    time_second_taxi_begin = 0;
    resetEncoders();
  }

  public void auto_state_periodic()
  {
    chassis.trans_high_with_brake();
    SmartDashboard.putNumber("AUTO_STATE", TWO_BALL_STATE);

    
    if (TWO_BALL_STOP_STATE == TWO_BALL_STATE) {
      DriverStation.reportWarning("Hit AutoState End Condition due to Dashboard selection of "+TWO_BALL_STATE, false);
      TWO_BALL_STATE = AutoState.AUTO_DONE;
    }
    
    // No matter what, in auto, always have the intake extended.
    intake.extendIntake();

    switch (TWO_BALL_STATE) {
      
      case AutoState.INITIAL: {
        intake.runIntake();
        if (time_intake_extended == 0) {
          time_intake_extended = System.currentTimeMillis();
        }


        // If we're in manual mode, then use time to figure this out. otherwise, use the limit switch.
        boolean is_intake_actually_extended = false;
        long wait_for_intake_extend_elapsed = System.currentTimeMillis() - time_intake_extended;
        /*
        if (catapultLimitSwitch.shouldIgnoreLimitSwitch()) {
          is_intake_actually_extended = (elapsed > INTAKE_EXTEND_WAIT);
        } else {
        */
          // Once the intakelimitswitch (which has a built-in-delay) is no longer pressed, it's safe to shoot
          is_intake_actually_extended = !intakeLimitSwitch.get();
        //}


        

        if (is_intake_actually_extended) { 
          TWO_BALL_STATE = AutoState.BEFORE_FIRST_SHOT_INTAKE_EXTENDED;
          break;
        } else {
          if (wait_for_intake_extend_elapsed > INTAKE_EXTEND_WAIT) {
            // This means that we didn't actually extend. There's a mechanical problem. The best we can do is Taxi.
            // Taxi only time!
            TWO_BALL_STATE = AutoState.AFTER_FIRST_SHOT_BEFORE_CATAPULT_DOWN;
            TWO_BALL_STOP_STATE = AutoState.AFTER_INTAKE_SECOND_CARGO;
            dontIntakeDuringAuto = true;
            intake.stopIntake();
          }
        }
        break;
      }
      case AutoState.BEFORE_FIRST_SHOT_INTAKE_EXTENDED: {
        intake.stopIntake();
        catapult.catapultPeriodic(true);
        if (time_first_shot_taken == 0) {
          time_first_shot_taken = System.currentTimeMillis();
        }
        // This delay is to account for the time required to run the choochoo
        //  before the actually shot itself happens.
        long elapsed = System.currentTimeMillis() - time_first_shot_taken;
        if (elapsed > FIRE_SHOT_WAIT) {
          TWO_BALL_STATE = AutoState.AFTER_FIRST_SHOT_BEFORE_CATAPULT_DOWN;
        }
        break;
      }
      case AutoState.AFTER_FIRST_SHOT_BEFORE_CATAPULT_DOWN: {
        catapult.catapultPeriodic(false);
        if (time_began_to_bring_down_catapult == 0) {
          time_began_to_bring_down_catapult = System.currentTimeMillis();
        }
        if (catapultLimitSwitch.isPhysicalSwitchPressed()) {
          TWO_BALL_STATE = AutoState.AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN;
        } else {
          long elapsed_catapult_down = System.currentTimeMillis() - time_began_to_bring_down_catapult;
          if (elapsed_catapult_down > TIME_BRING_DOWN_CATAPULT_WAIT) {
            dontIntakeDuringAuto = true;
            intake.stopIntake();
            TWO_BALL_STATE = AutoState.AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN;
            TWO_BALL_STOP_STATE = AutoState.AFTER_INTAKE_SECOND_CARGO;
          }
        }
        break;
      }
      case AutoState.AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN: {
        if (time_first_taxi_begin == 0) {
          time_first_taxi_begin = System.currentTimeMillis();
          resetEncoders();
        }
        // Ensure the robot doesn't go crazy. Stop after N seconds.
        long now = System.currentTimeMillis();
        long taxi_elapsed = now - time_first_taxi_begin;
        if (taxi_elapsed > TAXI_TIME_HARD_STOP) {
          DriverStation.reportError("Hit TAXI_TIME_HARD_STOP in AFTER_FIRST_SHOT_AFTER_CATAPULT_DOWN, elapsed="+taxi_elapsed+", time_first_taxi_begin="+time_first_taxi_begin+", now="+now, false);
          TWO_BALL_STATE = AutoState.AUTO_DONE;
          break;
        }

        if (dontIntakeDuringAuto) {
          intake.stopIntake();
        } else {
          intake.runIntake(ADDITIONAL_INTAKE_POWER);
        } 
        
        boolean arrived = autonStraightDrive.periodic(TAXI_DISTANCE, MAX_TAXI_SPEED);
        if (arrived) {
          time_first_taxi_begin = System.currentTimeMillis(); // no need to worry about taxi anymore.
          if (time_intake_began_after_arrival == 0) {
            time_intake_began_after_arrival = System.currentTimeMillis();
          }
          long elapsed = System.currentTimeMillis() - time_intake_began_after_arrival;
          if (elapsed >= INTAKE_CARGO_WAIT) {
            TWO_BALL_STATE = AutoState.AFTER_INTAKE_SECOND_CARGO;
            chassis.resetEncoders(); // To prepare for next journey.
            break;
          }
        }
        break;
      }
      case AutoState.AFTER_INTAKE_SECOND_CARGO: {
        if (dontIntakeDuringAuto) {
          intake.stopIntake();
        } else {
          intake.runIntake(ADDITIONAL_INTAKE_POWER); // Keep running the intake - we don't want to lose it,
                            // and we might not even fully have it in the claw yet!
        }
        if (time_return_taxi_begin == 0) {
          time_return_taxi_begin = System.currentTimeMillis();
        }
        // Ensure the robot doesn't go crazy. Hard stop after N seconds.
        long now = System.currentTimeMillis();
        long taxi_elapsed = now - time_return_taxi_begin;
        if (taxi_elapsed > TAXI_TIME_HARD_STOP) {
          DriverStation.reportError("Hit TAXI_TIME_HARD_STOP in AFTER_ITNAKE_SECOND_CARGO, elapsed="+taxi_elapsed+", time_return_taxi_begin="+time_return_taxi_begin+", now="+now, false);
          TWO_BALL_STATE = AutoState.AUTO_DONE;
          break;
        }

        boolean arrived = autonStraightDrive.periodic(RETURN_DISTANCE, MAX_TAXI_SPEED);
        if (arrived) {
          TWO_BALL_STATE = AutoState.AFTER_RETURN_TO_ORIGINAL_SPOT;
          chassis.resetEncoders(); // To prepare for next journey.
          break;
        }
        break;
      }
      case AutoState.AFTER_RETURN_TO_ORIGINAL_SPOT: {
        // At this point we can finally stop the intake.
        intake.stopIntake();
        catapult.catapultPeriodic(true);
        if (time_second_shot_taken == 0) {
          time_second_shot_taken = System.currentTimeMillis();
        }
        long elapsed = System.currentTimeMillis() - time_second_shot_taken;
        if (elapsed > FIRE_SHOT_WAIT) {
          TWO_BALL_STATE = AutoState.AUTO_DONE;
          //TWO_BALL_STATE = AutoState.AFTER_SECOND_SHOT;
          break;
        }
        break;
      }
      case AutoState.AFTER_SECOND_SHOT: {
        if (time_second_taxi_begin == 0) {
          time_second_taxi_begin = System.currentTimeMillis();
        }
        long now = System.currentTimeMillis();
        long taxi_elapsed = now - time_second_taxi_begin;
        // Ensure the robot doesn't go crazy. Hard stop after N seconds.
        if (taxi_elapsed > TAXI_TIME_HARD_STOP) {
          DriverStation.reportError("Hit TAXI_TIME_HARD_STOP in AFTER_SECOND_SHOT, elapsed="+taxi_elapsed+", time_second_taxi_begin="+time_second_taxi_begin+", now="+now, false);
          TWO_BALL_STATE = AutoState.AUTO_DONE;
          break;
        }

        catapult.catapultPeriodic(false);
        boolean arrived = autonStraightDrive.periodic(SECOND_TAXI_DISTANCE, MAX_TAXI_SPEED);
        if (arrived) {
          TWO_BALL_STATE = AutoState.AUTO_DONE;
          break;
        }
        break;
      }
      
      case AutoState.AUTO_DONE:
      default: {
        // Continue on this until it's time for Teleop!
        catapult.catapultPeriodic(false);
        chassis.stop();
        intake.stopIntake();

        break;
      }
    }

  }

  public void autonomousPeriodic()
  {
    chassis.runCompressor();
    auto_state_periodic();
  }

  /*

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
  */

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

  public void passivePeriodic()
  {
    intakeLimitSwitch.update();
    paintDashboard();
  }

  public void paintDashboard()
  {
    chassis.paintDashboard();
    catapult.paintDashboard();
    intake.paintDashboard();
    climber.paintDashboard();
  }
}
