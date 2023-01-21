// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
    public static class DrivetrainConstants{
    public static final int LEFT_WHEEL_PORT = 4;
    public static final int RIGHT_WHEEL_PORT = 3;
    public static final int LEFT_FOLLOWER_WHEEL_PORT = 31;
    public static final int RIGHT_FOLLOWER_WHEEL_PORT = 32;
    public static final double DRIVE_TRAIN_SPEED = .25;
    public static final int CONTROLLER_PORT = 0;
    public static final String ARCADE_DRIVE_STRING = "ARCADE";
    public static final String TANK_DRIVE_STRING = "TANK";
    public static final double WHEEL_RADIUS = Units.inchesToMeters(3.0);
    public static final double WHEEL_GEAR_RATIO = 1/12.75;
    public static final double DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION = (2*Math.PI*WHEEL_RADIUS*WHEEL_GEAR_RATIO);
    public static final int CURRENT_LIMIT = 40;
    public static final double MAX_SPEED = 0.5;
    public static final double SLOW_SPEED = 0.25;
    public static final double FAST_SPEED = 1;
  }
}
