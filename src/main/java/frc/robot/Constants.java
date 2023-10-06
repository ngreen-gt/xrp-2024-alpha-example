// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants {
    // XRP Motor IDs
    public static final int kLeftMotorPort = 0;
    public static final int kRightMotorPort = 1; 

    // Encoder ports
    public static final int[] kLeftEncoderPorts = {4,5};
    public static final int[] kRightEncoderPorts = {6,7};

    // Drive gear ratio and wheel diameter
    public static final double kGearRatio = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    public static final double kCountsPerMotorShaftRev = 12.0;
    public static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
    public static final double kWheelDiameterInches = 2.3622; // 60 mm
    public static final double kEncoderDistancePerPulse = (Math.PI * kWheelDiameterInches) / kCountsPerRevolution;
    public static final double kTrackwidthInches = 6.0;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthInches);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
