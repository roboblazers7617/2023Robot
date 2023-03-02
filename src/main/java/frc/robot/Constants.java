// Copywhite (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

  public enum ScoreLevel {
    LEVEL_1,
    LEVEL_2,
    LEVEL_3;
  }

  public enum PickupLocation {
    FLOOR,
    DOUBLE;
  }

  public enum PieceType {
    CONE,
    CUBE;
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final double DEADZONE = 0.1;
  }

  public static class DrivetrainConstants {

    public static final int GYRO_ID = 40;

    public static final int LEFT_WHEEL_ID = 3;
    public static final int RIGHT_WHEEL_ID = 4;
    public static final int LEFT_FOLLOWER_WHEEL_ID = 1;
    public static final int RIGHT_FOLLOWER_WHEEL_ID = 5;

    // public static final String ARCADE_DRIVE_STRING = "ARCADE";
    // public static final String TANK_DRIVE_STRING = "TANK";
    // public static final String CURVATURE_DRIVE_STRING = "CURVATURE";

    public static final double WHEEL_RADIUS = Units.inchesToMeters(3.0);
    public static final double WHEEL_GEAR_RATIO = 1.0 / 10.71;
    public static final double DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION = (2.0 * Math.PI * WHEEL_RADIUS
        * WHEEL_GEAR_RATIO);
    public static final double DRIVETRAIN_ENCODER_VELOCITY = (DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION / 60.0);

    public static final int CURRENT_LIMIT = 40;

    public static final double THE_NUMBER_3 = 7.0;

    public static final double REG_SPEED = 0.5;
    public static final double SLOW_SPEED = 0.25;
    public static final double FAST_SPEED = 1;
    public static final double MAX_ANGULAR_VELOCITY = 0.65;// .5
    public static final double MAX_LINEAR_VELOCITY = 0.7;// .5

    public static final double KP_LIN = 3.68;
    public static final double KI_LIN = 0.0;
    public static final double KD_LIN = 0.0;
    public static final double KS_LIN = .192;

    public static final double KP_ROT = 0.015;// 0.015
    public static final double KI_ROT = 0.0;
    public static final double KD_ROT = 0.0;
    public static final double KS_ROT = 0.3;

    public static final double KS = 0.192;
    public static final double KV = 4.0;
    public static final double KA = 0.424;

    public static final double SIMPLE_FF_LINEAR = 0.5;
    public static final double SIMPLE_FF_ANGULAR = 0.3;

    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(20.5);

    public static final double MAX_ERROR_LINEAR = Units.inchesToMeters(1.0);
    public static final double MAX_ERROR_ROTATION = 1.0; // in degrees
    // public static final double LINEAR_ERROR_TARGET_DRIVER =
    // Units.inchesToMeters(3);

    public static final double MAX_AUTO_ACCELERATION = 0.25;
    public static final double MAX_AUTO_VELOCITY = 0.4;

    public static final double RAMSETEb = 2.0;
    public static final double RAMSETEzeta = 0.7;

    public static final double DISTANCE_TO_SLOW = 24.0;

    public static final double RAMP_TIME_SECONDS = 0.001;

    public static final double ALLIANCE_BLUE_ROTATION = 180.0;
    public static final double ALLIANCE_RED_ROTATION = 0.0;

    
    public static final double LENGTH_OF_ROBOT = Units.inchesToMeters(31.0);
    public static final double X_OFFSET_FROM_SCORE_LOCATION = LENGTH_OF_ROBOT / 2.0 + Units.inchesToMeters(14.0);

    public static final double KP_BALANCE = 0.01;
    public static final double KI_BALANCE = 0;
    public static final double KD_BALANCE = 0;
    public static final double BALANCING_TOLERANCE = 1;
    // TODO: Lukas. (High) Decide this value
    public static final int MAX_BALANCE_SPEED = 0;

    public enum DrivetrainMode {
      arcadeDrive,
      tankDrive,
      curvatureDrive
    }

    public enum AutoPath {
      blueNodeOne("blue far 2 ball", true),
      blueNodeSix("blue mid 2 ball", true),
      blueNodeNine("blue near 2 ball", true),
      redNodeOne("red far 2 ball", true),
      redNodeSix("red mid 2 ball", true),
      redNodeNine("red near 2 ball", true),;

      private final String pathname;
      private final boolean isReverse;

      AutoPath(String pathname, boolean isReverse) {
        this.pathname = pathname;
        this.isReverse = isReverse;
      }
      public String pathname(){
        return pathname;
      }
      public boolean isReverse(){
        return isReverse;
      }
    }
  }

  public static class VisionConstants {
    // TODO: Lukas. (High) Confirm these location constants for camera
    public static final double CAMERA_PITCH = 0;
    public static final Transform3d CAMERA_POSITION = new Transform3d(new Translation3d(Units.inchesToMeters(17.5),
        Units.inchesToMeters(-.5), Units.inchesToMeters(6.875)), new Rotation3d(0, CAMERA_PITCH, 0));
    //TODO: Lukas. Is this used?
    public static final double[] TAG_HEIGHT = { 17.25, 17.25, 17.25, 17.25, 17.25, 17.25, 17.25, 17.25, 17.25, 17.25,
        17.25, 17.25, 17.25 };
    public static final double CAMERA_HEIGHT = (7.0 + (5.0 / 8.0));
    public static final String CAMERA_NAME = "eyeball";
  }

  public static class PnuematicsConstants {

    public static final int LEFT_ARM_PISTON_EXTEND_PORT = 11;
    public static final int LEFT_ARM_PISTON_RETRACT_PORT = 10;
    public static final int RIGHT_ARM_PISTON_EXTEND_PORT = 8;
    public static final int RIGHT_ARM_PISTON_RETRACT_PORT = 9;

    public enum PnuematicPositions {
      RETRACTED(Value.kReverse),
      EXTENDED(Value.kForward);

      Value mValue;

      PnuematicPositions(Value value) {
        mValue = value;
      }

      public Value getValue() {
        return mValue;
      }
    }
  }

  public static class ArmConstants {
    public static final int LIMIT_SWITCH_PORT = 0;
    public static final int SHOULDER_MOTOR_ID = 32;
    public static final AnalogInput SHOULDER_POTENTIOMETER_PORT = new AnalogInput(0);

    public static final double KP = 0; //THIS
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0;
    public static final double KG = 0.11; //THIS
    public static final double KV = 0;

    //TODO: Lukas. (High) Set a position tolerance
    public static final double POSITION_TOLERANCE = 0;
    public static final double SHOULDER_POTENTIOMETER_RANGE = 340;
    // TODO: need to find offset to paralell to floor 0
    public static final double SHOULDER_POTENTIOMETER_OFFSET = -222;
    public static final int CURRENT_LIMIT = 39;
    // TODO: Lukas. (High) Set gear ratio
    public static final double SHOULDER_GEAR_RATIO = 1.0/1.0; //THIS
    public static final double POSITION_CONVERSION_FACTOR = SHOULDER_GEAR_RATIO * 360.0; 
    public static final double MINIMUM_SHOULDER_ANGLE = 0; //THIS
    public static final double MAX_SHOULDER_ANGLE = 0.0;
    public static final double MAX_MANNUAL_WRIST_SPEED = 75.0; //THIS
    public static final double MAX_SPEED_DOWNWARD = -0.25; //THIS
    public static final double MAX_SPEED_UPWARD = 0.25; //THIS

    // TODO: Lukas. (High) Set the angles
    public enum ArmPositions {
      LEVEL_3(0, PnuematicPositions.EXTENDED),
      LEVEL_2(0, PnuematicPositions.RETRACTED),
      LEVEL_1(0, PnuematicPositions.RETRACTED),
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

  public static class WristConstants{
    public static final int WRIST_CAN_ID = 6;
    public static final int WRIST_LIMIT_SWITCH_CHANEL = 4;
    public static final int POT_CHANEL = 1;

    public static final int CURRENT_LIMIT = 20;

    public static final double WRIST_ANGLE_TOLERANCE = 1;
    public static final double MAX_WRIST_ANGLE = 103;
    public static final double MAX_UPWARD_WRIST_SPEED = 0.25;
    public static final double MAX_DOWNWARD_WRIST_SPEED = -0.1;
    public static final double MAX_WRIST_ACCEL = 0.12;
    public static final double WRIST_KS = 0.5;
    public static final double WRIST_KG = 0.2;
    public static final double WRIST_KV = 0;
    public static final double WRIST_KP = 0.011;
    public static final double WRIST_KI = 0.0;
    public static final double WRIST_KD = 0.0;
    public static final double WRIST_POT_OFFSET = -199;// so stowed is 120
    //TODO: Lukas. Can this be removed?
    public static final double WRIST_MANUAL_SLOWDOWN = .4;
    //TODO: Lukas. Can this be removed?
    public static final double MAX_APROACHING_WRIST_SPEED = .08;
    public static final double WRIST_GEAR_RATIO = 1.0/80.0;
    public static final double WRIST_ENCODER_CONVERSION_FACTOR = 360.0 * WRIST_GEAR_RATIO;
    public static final double MIN_WRIST_ANGLE = 10;
    public static final int WRIST_POT_SCALE = 340;

    public static final double MAX_MANNUAL_WRIST_SPEED = 50;

    // TODO: Lukas. (High) Determine these
    public enum WristPosition {
      STOW(WristConstants.MAX_WRIST_ANGLE),
      FLOOR_CUBE_PICKUP(0.06),
      FLOOR_CONE_PICKUP(0.07),
      DOUBLE_PICKUP(0.07),
      LEVEL_3(0.04),
      LEVEL_2(0.05),
      LEVEL_1(0.03);

      private final double angle;

      WristPosition(double angle) {
        this.angle = angle;
      }

      public double angle() {
        return angle;
      }
  }


  public static class IntakeConstants {
    public static final int INTAKE_CAN_ID = 7;
    public static final int DISTANCE_SENSOR_CHANEL = 5;
    public static final int INTAKE_LIMIT_SWITCH_ID = 2;
    public static final int CURRENT_LIMIT = 20;
    public static final double INTAKE_GEAR_RATIO = 1.0 / 5.0;
    public static final double INTAKE_ENCODER_CONVERSION_FACTOR = 360.0 * INTAKE_GEAR_RATIO;
    
    //TODO: Lukas. (High) Set these
    public enum IntakeDirection
    {
      STOP (0.0),
      PICK_CONE (0.75),
      PICK_CUBE (-0.5),
      PLACE_CONE (-0.5),
      PLACE_CUBE (0.75);

      private final double speed;

      IntakeDirection(double speed) {
        this.speed = speed;
      }

      public double speed() {
        return speed;
      }
    }


    }
  }

  
}