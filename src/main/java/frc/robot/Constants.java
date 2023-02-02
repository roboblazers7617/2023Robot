// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

  }
    public static class DrivetrainConstants{
    public static final int LEFT_WHEEL_PORT = 4;
    public static final int RIGHT_WHEEL_PORT = 3;
    public static final int LEFT_FOLLOWER_WHEEL_PORT = 31;
    public static final int RIGHT_FOLLOWER_WHEEL_PORT = 32;

    public static final double DRIVE_TRAIN_SPEED = .25;

    public static final String ARCADE_DRIVE_STRING = "ARCADE";
    public static final String TANK_DRIVE_STRING = "TANK";

    public static final double WHEEL_RADIUS = Units.inchesToMeters(3.0);
    public static final double WHEEL_GEAR_RATIO = 1.0/10.71;
    public static final double DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION = (2.0*Math.PI*WHEEL_RADIUS*WHEEL_GEAR_RATIO);
    public static final double DRIVETRAIN_ENCODER_VELOCITY = (DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION/60.0);

    public static final int CURRENT_LIMIT = 40;

    public static final double MAX_SPEED = 0.5;
    public static final double SLOW_SPEED = 0.25;
    public static final double FAST_SPEED = 1;

    public static final double KP_LIN = 0.5;
    public static final double KI_LIN = 0;
    public static final double KD_LIN = 0;
    public static final double KS_LIN = .25;

    public static final double KP_ROT = 0.015;
    public static final double KI_ROT = 0;
    public static final double KD_ROT = 0.0004;
    public static final double KS_ROT = 0.2;

    public static final double KS = 0.015;
    public static final double KV = 0.21;

    public static final int GYRO_ID = 40; 
    
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(27.0);
  }
   public static class VisionConstants {
    public static final Transform3d CAMERA_POSITION = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d()),
    new Pose3d(Units.inchesToMeters(16-(1+7/8)), 0, Units.inchesToMeters(2.25), new Rotation3d()));
    public static final double[] TAG_HEIGHT = {17.25,17.25,17.25,17.25,17.25,17.25,17.25,17.25,17.25,17.25,17.25,17.25,17.25};
    public static final double CAMERA_HEIGHT = 8;
    public static final double CAMERA_PITCH = 0;
   }
}
