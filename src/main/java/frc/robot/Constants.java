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
        public static final int BUTTON_BOX_CONTROLLER_PORT = 2;

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
        public static final double WHEEL_DIAMETER = 6.25; // in inches for calculations
        public static final double THE_NUMBER_THREE = 7.0;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(WHEEL_DIAMETER / 2);
        public static final double WHEEL_GEAR_RATIO = 1.0 / 10.75;
        public static final double DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION = (2.0 * Math.PI * WHEEL_RADIUS
                * WHEEL_GEAR_RATIO);
        public static final double DRIVETRAIN_ENCODER_VELOCITY = (DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION / 60.0);

        public static final int CURRENT_LIMIT = 40;

        public static final double REG_SPEED = 0.5;
        public static final double SLOW_SPEED = 0.25;
        public static final double FAST_SPEED = 1;
        public static final double MAX_ANGULAR_VELOCITY = 0.65;// .5
        public static final double MAX_LINEAR_VELOCITY = 0.7;// .5

        public static final double KP_LIN = 3.3174;
        public static final double KI_LIN = 0.0;
        public static final double KD_LIN = 0.0;
        public static final double KS_LIN = .12529;

        public static final double KP_ROT_POS = 0.015;// 0.015
        public static final double KI_ROT_POS = 0.0;
        public static final double KD_ROT_POS = 0.0;
        public static final double KS_ROT = 0.48693;
        public static final double KV_ROT = 2.9804;
        public static final double KA_ROT = 0.21385;

        public static final double KP_ROT_VEL = 0.54643;// 0.015
        public static final double KI_ROT_VEL = 0.0;
        public static final double KD_ROT_VEL = 0.0;

        public static final double KS = 0.10721;
        public static final double KV = 2.7934;
        public static final double KA = 0.274;

        public static final double SIMPLE_FF_LINEAR = 0.5;
        public static final double SIMPLE_FF_ANGULAR = 0.3;

        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(20.5);

        public static final double MAX_ERROR_LINEAR = Units.inchesToMeters(1.0);
        public static final double MAX_ERROR_ROTATION = 1.0; // in degrees
        // public static final double LINEAR_ERROR_TARGET_DRIVER =
        // Units.inchesToMeters(3);

        public static final double MAX_AUTO_ACCELERATION = 2;
        public static final double MAX_AUTO_VELOCITY = 3;

        public static final double RAMSETEb = 2.0;
        public static final double RAMSETEzeta = 0.7;

        public static final double DISTANCE_TO_SLOW = 24.0;

        public static final double RAMP_TIME_SECONDS = 0.001;

        public static final double ALLIANCE_BLUE_ROTATION = 180.0;
        public static final double ALLIANCE_RED_ROTATION = 0.0;

        public static final double LENGTH_OF_ROBOT = Units.inchesToMeters(38.5);
        public static final double X_OFFSET_FROM_SCORE_LOCATION = LENGTH_OF_ROBOT / 2.0 + Units.inchesToMeters(14.0);

        public static final double KP_BALANCE = 2;
        public static final double KI_BALANCE = 0;
        public static final double KD_BALANCE = 0.5;
        public static final double BALANCING_TOLERANCE = 2.5;
        // TODO: Lukas. (High) Decide this value
        public static final double MAX_BALANCE_SPEED = 0.550;// TODO: .5

        public enum DrivetrainMode {
            arcadeDrive,
            tankDrive,
            curvatureDrive
        }

        public enum AutoPath {
            blueNodeOne("blue far 2 ball", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            blueNodeSix("blue mid 2 ball", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, true, true),
            blueNodeNine("blue near 2 ball", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            redNodeOne("red far 2 ball", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            redNodeSix("red mid 2 ball", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, true, true),
            redNodeNine("red near 2 ball", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            leftBlueSimple("left blue simple", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            midBlueSimple("mid blue simple", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, true, true),
            rightBlueSimple("right blue simple", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            leftRedSimple("left red simple", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            midRedSimple("mid blue simple", true, PieceType.CONE, ScoreLevel.LEVEL_3, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, true, true),
            rightRedSimple("right red simple", true, PieceType.CONE, ScoreLevel.LEVEL_2, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            midleftBlueSimple("left blue simple", true, PieceType.CONE, ScoreLevel.LEVEL_2, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            midmidBlueSimple("mid blue simple", true, PieceType.CONE, ScoreLevel.LEVEL_2, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, true, true),
            midrightBlueSimple("right blue simple", true, PieceType.CONE, ScoreLevel.LEVEL_2, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            midleftRedSimple("left red simple", true, PieceType.CONE, ScoreLevel.LEVEL_2, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            midmidRedSimple("mid blue simple", true, PieceType.CONE, ScoreLevel.LEVEL_2, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, true, true),
            midrightRedSimple("right red simple", true, PieceType.CONE, ScoreLevel.LEVEL_2, ScoreLevel.LEVEL_2, PickupLocation.FLOOR, false, true),
            lowsimple("right red simple", true, PieceType.CUBE, ScoreLevel.LEVEL_1, ScoreLevel.LEVEL_1, PickupLocation.FLOOR, false, true),
            lowsimplebalance("mid blue simple", true, PieceType.CUBE, ScoreLevel.LEVEL_1, ScoreLevel.LEVEL_1, PickupLocation.FLOOR, true, true),
            autobalanceTest("autobalance test", false, PieceType.CUBE, ScoreLevel.LEVEL_1, ScoreLevel.LEVEL_1, PickupLocation.FLOOR, true, false);
      
      private final String pathname;
            private final boolean isReverse;
            private final PieceType selectedPiece;
            private final ScoreLevel ScoreLevelFirst;
            private final ScoreLevel ScoreLevelSecond;
            private final PickupLocation pickupLocation;
            private final boolean autoBalance;
            private final boolean scoring;

            AutoPath(String pathname, boolean isReverse, PieceType selectedPiece, ScoreLevel ScoreLevelFirst,
                    ScoreLevel ScoreLevelSecond, PickupLocation pickupLocation, boolean autoBalance, boolean scoring) {
                this.pathname = pathname;
                this.isReverse = isReverse;
                this.selectedPiece = selectedPiece;
                this.ScoreLevelFirst = ScoreLevelFirst;
                this.ScoreLevelSecond = ScoreLevelSecond;
                this.pickupLocation = pickupLocation;
                this.autoBalance = autoBalance;
                this.scoring = scoring;
            }

            public String pathname() {
                return pathname;
            }

            public boolean isReverse() {
                return isReverse;
            }

            public PieceType selectedPiece() {
                return selectedPiece;
            }

            public ScoreLevel scoreLevelFirst() {
                return ScoreLevelFirst;
            }

            public ScoreLevel scoreLevelSecond() {
                return ScoreLevelSecond;
            }

            public PickupLocation pickupLocation() {
                return pickupLocation;
            }

            public boolean autoBalance() {
                return autoBalance;
            }

            public boolean scoring() {
                return scoring;
            }
        }
    }

    public static class VisionConstants {
        // TODO: Lukas. (High) Confirm these location constants for camera
        public static final double CAMERA_PITCH = 0;
        public static final Transform3d CAMERA_POSITION = new Transform3d(
                new Translation3d(Units.inchesToMeters(-5.8125),
                        Units.inchesToMeters(0), Units.inchesToMeters(30.5)),
                new Rotation3d(0, CAMERA_PITCH, 0));
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
        public static final int SHOULDER_MOTOR_ID = 31;
        public static final int SHOULDER_FOLLOWER_MOTOR_ID = 32;
        public static final AnalogInput SHOULDER_POTENTIOMETER_PORT = new AnalogInput(0);

        public static final double KP = 0.07;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0.0;
        public static final double KG = 0.01;
        public static final double KV = 0;

        // TODO: Lukas. (High) Set a position tolerance
        public static final double POSITION_TOLERANCE = 0.5;
        public static final double SHOULDER_POTENTIOMETER_RANGE = 340;
        public static final double SHOULDER_POTENTIOMETER_OFFSET = -222;
        public static final int CURRENT_LIMIT = 39;
        public static final double SHOULDER_GEAR_RATIO = 1.0 / 204;
        public static final double POSITION_CONVERSION_FACTOR = SHOULDER_GEAR_RATIO * 360.0;
        public static final double MINIMUM_SHOULDER_ANGLE = -51;
        public static final double MAX_SHOULDER_ANGLE = 50;
        public static final double MAX_MANNUAL_ARM_SPEED = 50.0;
        public static final double MAX_SPEED_DOWNWARD = -0.5;
        public static final double MAX_SPEED_UPWARD = 0.3;
        public static final double PISTON_BACK = -90 - MINIMUM_SHOULDER_ANGLE;
        public static final double PISTON_FORWARD = 0;

        // TODO: Lukas. (High) Set the angles
        public enum ArmPositions {
            LEVEL_3_CONE(40, PnuematicPositions.EXTENDED),
            LEVEL_2_CONE(4, PnuematicPositions.RETRACTED),
            LEVEL_1_CONE(MINIMUM_SHOULDER_ANGLE, PnuematicPositions.RETRACTED),
            LEVEL_3_CUBE(37, PnuematicPositions.EXTENDED), // TODO
            LEVEL_2_CUBE(-27, PnuematicPositions.RETRACTED), // TODO
            LEVEL_1_CUBE(MINIMUM_SHOULDER_ANGLE, PnuematicPositions.RETRACTED), // TODO
            STOW(MINIMUM_SHOULDER_ANGLE, PnuematicPositions.RETRACTED),
            FLOOR_PICKUP_CONE(-42, PnuematicPositions.EXTENDED),
            FLOOR_PICKUP_CUBE(-39, PnuematicPositions.EXTENDED),
            STATION_PICKUP_CONE(17.0, PnuematicPositions.RETRACTED),
            STATION_PICKUP_CUBE(10.5, PnuematicPositions.RETRACTED);

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

    public static class WristConstants {
        public static final int WRIST_CAN_ID = 6;
        public static final int WRIST_LIMIT_SWITCH_CHANEL = 4;
        public static final int POT_CHANEL = 1;

        public static final int CURRENT_LIMIT = 20;

        public static final double WRIST_ANGLE_TOLERANCE = 5;
        public static final double MAX_WRIST_ANGLE = 103;
        public static final double MAX_UPWARD_WRIST_SPEED = 0.33; // TODO: Changed 3/4/23 from 0.25
        public static final double MAX_DOWNWARD_WRIST_SPEED = -0.2;
        public static final double MAX_WRIST_ACCEL = 0.12;
        public static final double WRIST_KS = 0.5;
        public static final double WRIST_KG = 0.35;
        public static final double WRIST_KV = 0;
        public static final double WRIST_KP = 0.011;
        public static final double WRIST_KI = 0.0;
        public static final double WRIST_KD = 0.0;
        public static final double WRIST_POT_OFFSET = -199;// so stowed is 120
        public static final double WRIST_GEAR_RATIO = 1.0 / 80.0;
        public static final double WRIST_ENCODER_CONVERSION_FACTOR = 360.0 * WRIST_GEAR_RATIO;
        public static final double MIN_WRIST_ANGLE = -42;
        public static final int WRIST_POT_SCALE = 340;

        public static final double MAX_MANNUAL_WRIST_SPEED = 65;

        public enum WristPosition {
            STOW(WristConstants.MAX_WRIST_ANGLE),
            FLOOR_CUBE_PICKUP(36),
            FLOOR_CONE_PICKUP(39),
            DOUBLE_PICKUP_CONE(-15),
            DOUBLE_PICKUP_CUBE(-3.5),
            LEVEL_3_CONE(-9),
            LEVEL_2_CONE(-8),
            LEVEL_1_CONE(41),
            LEVEL_3_CUBE(21), // TODO
            LEVEL_2_CUBE(79), // TODO
            LEVEL_1_CUBE(WristConstants.MAX_WRIST_ANGLE);// TODO

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
            public static final double CUBE_SENSOR_LIMIT = 600;
            public static final double CONE_CURRENT_LIMIT = 30.0;

            public static final double KP = 0.01;
            public static final double KI = 0.0;
            public static final double KD = 0.0;

            public static final double MAX_PID_SPEED = 0.50;

            public enum IntakeDirection {
                STOP(0.0),
                PICK_CONE(-0.75),
                PICK_CUBE(0.5),
                PLACE_CONE(0.5),
                PLACE_CUBE(-1);

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