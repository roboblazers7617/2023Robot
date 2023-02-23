package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DrivetrainConstants;

public class FieldPositions {
    public enum FieldLocation {
        NODE1,
        NODE2,
        NODE3,
        NODE4,
        NODE5,
        NODE6,
        NODE7,
        NODE8,
        NODE9,
        DOUBLE_STATION,
        SINGLE_STATION,
        AUTO_LOCATION_1,
        AUTO_LOCATION_2,
        AUTO_LOCATION_3,
        AUTO_LOCATION_4;
      }
    
      private static final Map<FieldLocation, Pose2d> BLUE_MAP = Map.ofEntries(
    
        Map.entry(FieldLocation.NODE1, new Pose2d(new Translation2d(Units.inchesToMeters(40.45) + DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(19.875)), Rotation2d.fromDegrees(180))),
    
        Map.entry(FieldLocation.NODE2, new Pose2d(new Translation2d(Units.inchesToMeters(40.45) + DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(41.875)), Rotation2d.fromDegrees(180))),
    
        Map.entry(FieldLocation.NODE3, new Pose2d(new Translation2d(Units.inchesToMeters(40.45) + DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(63.875)), Rotation2d.fromDegrees(180))),
    
        Map.entry(FieldLocation.NODE4, new Pose2d(new Translation2d(Units.inchesToMeters(40.45) + DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(85.875)), Rotation2d.fromDegrees(180))),
    
        Map.entry(FieldLocation.NODE5, new Pose2d(new Translation2d(Units.inchesToMeters(40.45) + DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(107.875)), Rotation2d.fromDegrees(180))),
    
        Map.entry(FieldLocation.NODE6, new Pose2d(new Translation2d(Units.inchesToMeters(40.45) + DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(129.875)), Rotation2d.fromDegrees(180))),
    
        Map.entry(FieldLocation.NODE7, new Pose2d(new Translation2d(Units.inchesToMeters(40.45) + DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(151.875)), Rotation2d.fromDegrees(180))),
    
        Map.entry(FieldLocation.NODE8, new Pose2d(new Translation2d(Units.inchesToMeters(40.45) + DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(173.875)), Rotation2d.fromDegrees(180))),
    
        Map.entry(FieldLocation.NODE9, new Pose2d(new Translation2d(Units.inchesToMeters(40.45) + DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(195.875)), Rotation2d.fromDegrees(180))),
    
        Map.entry(FieldLocation.DOUBLE_STATION, new Pose2d(new Translation2d(Units.inchesToMeters(636.96) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(265.74)), Rotation2d.fromDegrees(0))),
    
        Map.entry(FieldLocation.SINGLE_STATION, new Pose2d(new Translation2d(Units.inchesToMeters(557.635), Units.inchesToMeters(315.5) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION), Rotation2d.fromDegrees(90)))
    
    );
    
     
    
    private static final Map<FieldLocation, Pose2d> RED_MAP = Map.ofEntries(
    
      Map.entry(FieldLocation.NODE1, new Pose2d(new Translation2d(Units.inchesToMeters(610.77) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(19.875)), Rotation2d.fromDegrees(0))),
    
      Map.entry(FieldLocation.NODE2, new Pose2d(new Translation2d(Units.inchesToMeters(610.77) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(41.875)), Rotation2d.fromDegrees(0))),
    
      Map.entry(FieldLocation.NODE3, new Pose2d(new Translation2d(Units.inchesToMeters(610.77) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(63.875)), Rotation2d.fromDegrees(0))),
    
      Map.entry(FieldLocation.NODE4, new Pose2d(new Translation2d(Units.inchesToMeters(610.77) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(85.875)), Rotation2d.fromDegrees(0))),
    
      Map.entry(FieldLocation.NODE5, new Pose2d(new Translation2d(Units.inchesToMeters(610.77) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(107.875)), Rotation2d.fromDegrees(0))),
    
      Map.entry(FieldLocation.NODE6, new Pose2d(new Translation2d(Units.inchesToMeters(610.77) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(129.875)), Rotation2d.fromDegrees(0))),
    
      Map.entry(FieldLocation.NODE7, new Pose2d(new Translation2d(Units.inchesToMeters(610.77) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(151.875)), Rotation2d.fromDegrees(0))),
    
      Map.entry(FieldLocation.NODE8, new Pose2d(new Translation2d(Units.inchesToMeters(610.77) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(173.875)), Rotation2d.fromDegrees(0))),
    
      Map.entry(FieldLocation.NODE9, new Pose2d(new Translation2d(Units.inchesToMeters(610.77) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(195.875)), Rotation2d.fromDegrees(0))),
    
      Map.entry(FieldLocation.DOUBLE_STATION, new Pose2d(new Translation2d(Units.inchesToMeters(14.25 )+ DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION, Units.inchesToMeters(265.74)), Rotation2d.fromDegrees(180))),
    
      Map.entry(FieldLocation.SINGLE_STATION, new Pose2d(new Translation2d(Units.inchesToMeters(93.645), Units.inchesToMeters(315.5) - DrivetrainConstants.X_OFFSET_FROM_SCORE_LOCATION), Rotation2d.fromDegrees(90)))
    
    );
    
     
    
    public static final Map<Alliance, Map<FieldLocation, Pose2d>> POSE_MAP = Map.of(
        Alliance.Blue, BLUE_MAP,
        Alliance.Red, RED_MAP
    );

    public Translation2d getTargetTranslation(FieldLocation targetLocation, Alliance color){
        Pose2d myPose = POSE_MAP.get(color).get(targetLocation);
        return myPose.getTranslation();
      }
}
