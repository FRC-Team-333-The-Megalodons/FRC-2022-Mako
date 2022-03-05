// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean xboxDrive = false;
    public static final boolean twoDriverMode = false;

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
        public static final int HOLDER_ID = 0;

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
        public static final int LIMIT_SWITCH = 9;

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
        public static final int INTAKE_RUN = 2;//op
        public static final int INTAKE_REVERSE = 5;//op
        public static final int INTAKE_FORWARD = 4;//op
        public static final int INTAKE_BACK = 3;//op
        public static final int LOW_GEAR = 7;//driver
        public static final int HIGH_GEAR = 8;//driver
        public static final int CLIMBER_UP = 11;//op
        public static final int CLIMBER_DOWN = 12;//op
        public static final int CATAPULT_DOWN = 10;//driver
    }
}
