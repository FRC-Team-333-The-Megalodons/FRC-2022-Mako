// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean twoDriverMode = true;

    public static class DeviceIDs {

        //chassis IDS
        public static final int LEFT_LEADER_ID = 1;
        public static final int LEFT_FOLLOWER_ID = 2;
        public static final int LEFT_FOLLOWER2_ID = 3;

        public static final int RIGHT_LEADER_ID = 4;
        public static final int RIGHT_FOLLOWER_ID = 5;
        public static final int RIGHT_FOLLOWER2_ID = 6;

        //intake ID
        public static final int INTAKE_MOTOR_ID = 10;
        public static final int HOLDER_ID = 11;

        // catapult ID
        public static final int CATAPULT_ID = 7;

        // climber ID
        public static final int LEFT_CLIMBER_ID = 8;
        public static final int RIGHT_CLIMBER_ID = 9;
        
        //joysticks
        public static final int JOYSTICK_PORT = 0;
        public static final int CONTROLLER_PORT = 1;

        //solenoids
        public static final int INTAKE_SOLENOID1 = 0;
        public static final int INTAKE_SOLENOID2 = 1;
        //public static final int CATAPULT_SOLENOID = 2;
        public static final int DRIVETRAIN_SOLENOID_LOW = 6;
        public static final int DRIVETRAIN_SOLENOID_HIGH = 7;

        // Modules
        public static final int POWER_DISTRIBUTION_BOARD_PORT = 24;
        public static final int PNEMATIC_HUB = 22;

        // Sensors
        public static final int LIMIT_SWITCH = 0;

    }

    public static class JoyStickButtons {
        // buttons for Thrustmaster
        /*public static final int INTAKE_RUN = 2;
        public static final int INTAKE_REVERSE = 5;
        public static final int INTAKE_FORWARD = 3;
        public static final int INTAKE_BACK = 4;
        public static final int CATAPULT = 1;
        public static final int CATAPULT_DOWN = 10;
        public static final int LOW_GEAR = 8;
        public static final int HIGH_GEAR = 9;
        public static final int CLIMBER_UP = 14;
        public static final int CLIMBER_DOWN = 15;*/

        //buttons for x-pro
        public static final int CATAPULT = 1;//driver
        public static final int LOW_GEAR = 2; //driver
        public static final int INTAKE_RUN = 2;//op
        public static final int INTAKE_REVERSE = 5;//op
        public static final int INTAKE_FORWARD = 3;//op
        public static final int INTAKE_BACK = 4;//op
        public static final int LOW_GEAR_EXTRA1 = 7;//driver
        public static final int LOW_GEAR_EXTRA2 = 8;//driver
        public static final int CLIMBER_UP = 11;//op
        public static final int CLIMBER_DOWN = 12;//op
        public static final int CATAPULT_DOWN = 10;//driver
    }

    public static class ControllerButtons {
        // Sometimes the controller thinks it's a joystick
        public static final int LEFT_TRIGGER = 7;
        public static final int RIGHT_TRIGGER = 8;
    }


    public static class RobotValues {
        // Below values are from the CHaracterization tool
        public static final double ksVolts = 0.19804;
        public static final double kvVoltSecondsPerMeter = 0.26735;
        public static final double kaVoltSecondsSquaredPerMeter = 0.075001;

        public static final double kPDriveVel = 3.5898;

        public static final double kTrackwidthMeters = 5.3367;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Below values are from manual testing with pushing the robot the length of a 24" stick. Note that Eman was on the right side of the robot, so the Right Leader will be most precise
        // First run:   Left Leader = 8.619077, Right Leader = -8.785745
        // Second run:  Left Leader = 8.500028, Right Leader = -8.666696
        // Third run:   Left Leader = 8.357168, Right Leader = -8.761935
        // Average: Left Leader = 8.492091, Right Leader = 8.738125333333
        public static final double k_24inches_to_meters = 0.0254*24;
        public static final double k_hallEffectUnitsPer24Inches = 8.74;
        public static final double kHallEffectUnitsPerMeter = 8.74/k_24inches_to_meters;
    }
}
