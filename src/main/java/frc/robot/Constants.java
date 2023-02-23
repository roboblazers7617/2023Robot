// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.PnuematicsConstants.PnuematicPositions;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * 
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

  }

  public static class DrivetrainConstants {
    public static final int LEFT_WHEEL_PORT = 4;
    public static final int RIGHT_WHEEL_PORT = 3;
    public static final int LEFT_FOLLOWER_WHEEL_PORT = 31;
    public static final int RIGHT_FOLLOWER_WHEEL_PORT = 32;

    public static final double DRIVE_TRAIN_SPEED = .25;

    public static final String ARCADE_DRIVE_STRING = "ARCADE";
    public static final String TANK_DRIVE_STRING = "TANK";

    public static final double WHEEL_RADIUS = Units.inchesToMeters(3.0);
    public static final double WHEEL_GEAR_RATIO = 1.0 / 10.71;
    public static final double DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION = (2.0 * Math.PI * WHEEL_RADIUS
        * WHEEL_GEAR_RATIO);
    public static final double DRIVETRAIN_ENCODER_VELOCITY = (DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION / 60.0);

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

    private final Map<FieldLocation, Pose2d> BLUE_MAP = Map.ofEntries(Map.entry(FieldLocation.NODE_1, new Pose2d()),
    Map.entry(FieldLocation.NODE_2, new Pose2d()), Map.entry(FieldLocation.NODE_3, new Pose2d()),
    Map.entry(FieldLocation.NODE_4, new Pose2d()), Map.entry(FieldLocation.NODE_5, new Pose2d()),
    Map.entry(FieldLocation.NODE_6, new Pose2d()), Map.entry(FieldLocation.NODE_7, new Pose2d()),
    Map.entry(FieldLocation.NODE_8, new Pose2d()), Map.entry(FieldLocation.NODE_9, new Pose2d()),
    Map.entry(FieldLocation.DOUBLE_SUBSTATION, new Pose2d()),
    Map.entry(FieldLocation.SINGLE_SUBSTATION, new Pose2d()));

private final Map<FieldLocation, Pose2d> RED_MAP = Map.ofEntries(Map.entry(FieldLocation.NODE_1, new Pose2d()),
    Map.entry(FieldLocation.NODE_2, new Pose2d()), Map.entry(FieldLocation.NODE_3, new Pose2d()),
    Map.entry(FieldLocation.NODE_4, new Pose2d()), Map.entry(FieldLocation.NODE_5, new Pose2d()),
    Map.entry(FieldLocation.NODE_6, new Pose2d()), Map.entry(FieldLocation.NODE_7, new Pose2d()),
    Map.entry(FieldLocation.NODE_8, new Pose2d()), Map.entry(FieldLocation.NODE_9, new Pose2d()),
    Map.entry(FieldLocation.DOUBLE_SUBSTATION, new Pose2d()),
    Map.entry(FieldLocation.SINGLE_SUBSTATION, new Pose2d()));

private final Map<Alliance, Map<FieldLocation, Pose2d>> POSE_MAPS = Map
    .ofEntries(Map.entry(Alliance.Blue, BLUE_MAP), Map.entry(Alliance.Red, RED_MAP));
  }
  private enum FieldLocation {
    NODE_1(),
    NODE_2(),
    NODE_3(),
    NODE_4(),
    NODE_5(),
    NODE_6(),
    NODE_7(),
    NODE_8(),
    NODE_9(),
    DOUBLE_SUBSTATION(),
    SINGLE_SUBSTATION();

    FieldLocation() {
    }
  }

  public static class VisionConstants {
    public static final Transform3d CAMERA_POSITION = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d()),
        new Pose3d(Units.inchesToMeters(16 - (1 + 7 / 8)), 0, Units.inchesToMeters(2.25), new Rotation3d()));
    public static final double[] TAG_HEIGHT = { 17.25, 17.25, 17.25, 17.25, 17.25, 17.25, 17.25, 17.25, 17.25, 17.25,
        17.25, 17.25, 17.25 };
    public static final double CAMERA_HEIGHT = 8;
    public static final double CAMERA_PITCH = 0;
  }

  public static class PnuematicsConstants{

    public static final int LEFT_ARM_PISTON_EXTEND_PORT = 0;
    public static final int LEFT_ARM_PISTON_RETRACT_PORT = 0;
    public static final int RIGHT_ARM_PISTON_EXTEND_PORT = 0;
    public static final int RIGHT_ARM_PISTON_RETRACT_PORT = 0;

    public enum PnuematicPositions{
      RETRACTED(Value.kReverse),
      EXTENDED(Value.kForward);
      Value mValue;
      PnuematicPositions(Value value){
        mValue = value;
      }
      public Value getValue() {
          return mValue;
      }
    }
  }

  public static class ArmConstants {

    public static final int SHOULDER_MOTOR_PORT = 0;
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final AnalogInput SHOULDER_POTENTIOMETER_PORT = new AnalogInput(0);
    public static final AnalogInput WRIST_POTENTIOMETER_PORT = new AnalogInput(1);
    public static final double MAX_SHOULDER_VELOCITY = 0;
    public static final double MAX_SHOULDER_ACCELERATION = 0;
    public static final double KS = 0;
    public static final double KG = 0;
    public static final double KV = 0;
    public static final double POSITION_TOLERANCE = 0;
    public static final int LIMIT_SWITCH_PORT = 0;
    public static final double UPPER_ANGLE_LIMIT = 0;
    public static final double MAX_SPEED = 0;

    public enum ArmPositions {
      HIGH_GOAL(0, PnuematicPositions.EXTENDED),
      MID_GOAL(0, PnuematicPositions.RETRACTED),
      LOW_GOAL(0, PnuematicPositions.RETRACTED),
      STOW(0, PnuematicPositions.RETRACTED),
      FLOOR_PICKUP(0, PnuematicPositions.RETRACTED),
      STATION_PICKUP(0, PnuematicPositions.EXTENDED);

      private final double shoulderAngle;
      private final PnuematicPositions pistonPosition;

      ArmPositions(double shoulderAngle, PnuematicPositions pistonPosition) {
        this.shoulderAngle = shoulderAngle;
        this.pistonPosition = pistonPosition;
      }

      public double getShoulderAngle() {
        return shoulderAngle;
      }
      public PnuematicPositions getPistonPosition() {
          return pistonPosition;
      }
    }
  }



   public static class IntakeConstants {
    public static final int WRIST_CAN_ID = 22;
    public static final int INTAKE_CAN_ID = 23;
    public static final int POT_CHANEL = 1;
    public static final int DISTANCE_SENSOR_CHANEL = 5;
    public static final int INTAKE_LIMIT_SWITCH_ID = 2;
    public static final int WRIST_POT_SCALE = 270;
    public static final int WRIST_LIMIT_SWITCH_CHANEL = 4;
    public static final int CURRENT_LIMIT = 40;
    public static final double MAX_WRIST_ANGLE = 2.094;
    public static final double MAX_WRIST_SPEED = 0.25;
    public static final double MAX_WRIST_ACCEL = 0.12;
    public static final double WRIST_KS = 0.5;
    public static final double WRIST_KG = 0.5;
    public static final double WRIST_KV = 0.5;
    public static final double WRIST_KP = 2.0;
    public static final double WRIST_KI = 0.0;
    public static final double WRIST_KD = 0.0;
    public static final double WRIST_ANGLE_TOLERANCE = 0.1;
    public enum IntakeDirection
    {
      Stop (0.0),
      PickCone (0.25),
      PickCube (-0.25),
      PlaceCone (-0.25),
      PlaceCube (0.25);

      private final double speed;
      IntakeDirection (double speed) {
        this.speed = speed;
      }
      public double speed(){
        return speed;
      }
    } 
    public enum WristPosition
    {
      Store (0.0),
      FloorCubePickup (0.06),
      FloorConePickup ( 0.07),
      StationPickup(0.07),
      LevelThreePlace (0.04),
      LevelTwoPlace (0.05),
      LevelOnePlace (0.03);


      private final double angle;
      WristPosition (double angle) {
        this.angle = angle;
      }
      public double angle(){
        return angle;
      }


   }
  }

}
