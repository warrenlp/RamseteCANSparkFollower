/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final class DriveConstants
    {
        public static final int LEFT_MASTER_MOTOR = 8;
        public static final int LEFT_SLAVE_MOTOR = 10;
        public static final int RIGHT_MASTER_MOTOR = 6;
        public static final int RIGHT_SLAVE_MOTOR = 11;

        public static final double TRACK_WIDTH_METERS = 0.648;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
                new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

        public static final double GEAR_RATIO = 10.71;
        public static final double WHEEL_DIAMETER_METERS = 0.1905;
        public static final double ENCODER_DISTANCE =
                // Assumes the encoders are directly mounted on the wheel shafts
                (WHEEL_DIAMETER_METERS * Math.PI) / GEAR_RATIO;

        public static final double RPM_TO_MPS = ENCODER_DISTANCE / 60.0;
        public static final double MPS_TO_RPM = 1.0 / RPM_TO_MPS;

        public static final boolean GYRO_REVERSED = true;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
//        public static final double S_VOLTS = 0.22;
        public static final double S_VOLTS = 0.116;
//        public static final double V_VOLT_SECONDS_PER_METER = 1.98;
        public static final double V_VOLT_SECONDS_PER_METER = 2.32;
//        public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.2;
        public static final double A_VOLT_SECONDS_SQUARED_PER_METER = 0.4;
    }

    public static final class OIConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static final class AutoConstants
    {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double RAMSETE_B = 2;
//        public static final double RAMSETE_B = 10;
        public static final double RAMSETE_ZETA = 0.7;
//        public static final double RAMSETE_ZETA = 7.0;
    }
}
