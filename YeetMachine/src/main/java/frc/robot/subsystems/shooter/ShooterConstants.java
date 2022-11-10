// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class ShooterConstants {

    public final static int leftNeo = 11;
    public final static int rightNeo = 12;
    public final static double shooterPower = 1;

    public static class HoodConstants{

        public final static int hoodNeo = 13;

        public final static double HOME = 0;
        public final static double FENDER = 0;
        public final static double TAXI = 0;
        public final static double LAUNCHPAD = 34;

        public final static double kP = .005;
        public final static double kI = .0001;
        public final static double kD = 0;
        public final static double kILIM = 4;
    }

    public static class IndexerConstants{
        public final static int motor1 = 0;
        public final static int motor2 = 0;
    }
}
