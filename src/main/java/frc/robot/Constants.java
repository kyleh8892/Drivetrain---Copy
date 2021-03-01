// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.2;
    public static final double kvVoltSecondsPerMeter = 2.39;
    public static final double kaVoltSecondsSquaredPerMeter = 0.453;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = .1;

    public static final double kTrackwidthMeters = .5588;

    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = .5;
    public static final double kRamseteZeta = 0.7;


    public static final int kLeftFrontMotor = 1;
    public static final int kLeftBackMotor = 2;

    public static final int kRightFrontMotor = 11;
    public static final int kRightBackMotor = 12;

    public static final double kDistancePerRotation = 0.053848;
        
}
