// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int PPR = 2048;
  public static final double STEERING_GR = 150.0/7;
  public static final double DRIVE_GR = 8.14;
  public static final double WHEEL_RADIUS_METERS = 0.0508;
  public static final double SWERVE_MODULE_OFFSET = 0.3675;
  public static final double MAX_SPEED_MPS = 5;

}
