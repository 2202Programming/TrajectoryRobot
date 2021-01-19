/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
        // Drivetrain
        public static final int FL_SMAX = 30;
        public static final int ML_SMAX = 31;
        public static final int BL_SMAX = 32;
        public static final int FR_SMAX = 33;
        public static final int MR_SMAX = 34;
        public static final int BR_SMAX = 35;

        public static final double LOW_GEAR_RATIO = 18.0;
        public static final double WHEEL_DIAMETER = 0.5; //in feet

            /**
     * CAN bus IDs
     * 
     * Please keep in order ID order
     * 
     */
    public static final class CAN {
        // CAN ID for non-motor devices
        public static final int PDP = 0; // this must be 0
        public static final int PCM1 = 1; // default ID for PCM
        public static final int PCM2 = 2;

        // Lidar 
        public static final int FRONT_RIGHT_LIDAR = 21;
        public static final int FRONT_LEFT_LIDAR = 22;
       
        // Drivetrain
        public static final int FL_SMAX = 30;
        public static final int ML_SMAX = 31;
        public static final int BL_SMAX = 32;
        public static final int FR_SMAX = 33;
        public static final int MR_SMAX = 34;
        public static final int BR_SMAX = 35;
    }

        // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    
    public static final double ksVolts = 0.105; //updated
    public static final double kvVoltSecondsPerMeter = 1.49; //updated
    public static final double kaVoltSecondsSquaredPerMeter = 0.122; //updated

    public static final double kPDriveVel = 2.25; //updated
    public static final double kTrackwidthMeters = 1.462; //updated

    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
}
