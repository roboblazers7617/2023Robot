// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.IntakeConstants.IntakeDirection;
import frc.robot.Constants.IntakeConstants.WristPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class armIntakeCordinatorUtil {

    public armIntakeCordinatorUtil(){}
    public enum ScoreLevel {
        LEVEL_1(ArmPositions.LOW_GOAL, WristPosition.LevelOnePlace),
        LEVEL_2(ArmPositions.MID_GOAL, WristPosition.LevelTwoPlace),
        LEVEL_3(ArmPositions.HIGH_GOAL, WristPosition.LevelThreePlace);

        private ArmPositions armPosition;
        private WristPosition wristPosition;

        ScoreLevel(ArmPositions armPosition, WristPosition wristPosition) {
            this.armPosition = armPosition;
            this.wristPosition = wristPosition;
        }

        public ArmPositions getArmPosition() {
            return armPosition;
        }

        public WristPosition getWristPosition() {
            return wristPosition;
        }
    }

    public enum PickupLocation {
        FLOOR_CUBE(ArmPositions.FLOOR_PICKUP, WristPosition.FloorCubePickup, IntakeDirection.PickCube),
        DOUBLE_CUBE(ArmPositions.STATION_PICKUP, WristPosition.StationPickup, IntakeDirection.PickCube),
        FLOOR_CONE(ArmPositions.FLOOR_PICKUP, WristPosition.FloorConePickup, IntakeDirection.PickCone),
        DOUBLE_CONE(ArmPositions.STATION_PICKUP, WristPosition.StationPickup, IntakeDirection.PickCone);

        private ArmPositions armPosition;
        private WristPosition wristPosition;
        private IntakeDirection intakeDirection;

        PickupLocation(ArmPositions armPosition, WristPosition wristPosition, IntakeDirection intakeDirection) {
            this.armPosition = armPosition;
            this.wristPosition = wristPosition;
            this.intakeDirection = intakeDirection;
        }
        public ArmPositions getArmPosition() {
            return armPosition;
        }
        public IntakeDirection getIntakeDirection() {
            return intakeDirection;
        }
        public WristPosition getWristPosition() {
            return wristPosition;
        }

    }

    public enum PickupPlaces{
        FLOOR(),
        DOUBLE();

        PickupPlaces(){}
    }

    public enum PieceType {
        CONE(),
        CUBE();

        PieceType() {
        }
    }

    private PieceType desiredOrHeldPiece = null;


    
    private ScoreLevel desiredScoreLevel = null;

    private Pose2d desiredScoreLocation;
    private Pose2d desiredPickupLocation;

    private Arm mArm;
    private armIntakeCordinatorUtil mCordinatorUtil;

        
    public ArmPositions getScoreArmPosition() {
        return (desiredScoreLevel != null) ? desiredScoreLevel.getArmPosition() : null;
    }
    public WristPosition getScoreWristPosition() {
        return (desiredScoreLevel != null) ? desiredScoreLevel.getWristPosition() : null;
    }

    public void setDesiredScoreLevel(ScoreLevel level) {
        desiredScoreLevel = level;
    }



    public void setDesiredOrHeldPiece(PieceType mDesiredOrHeldPiece) {
        desiredOrHeldPiece = mDesiredOrHeldPiece;
    }

    public PieceType getDesiredOrHeldPiece() {
        return desiredOrHeldPiece;
    }

    public void setDesiredPickupLocation(Pose2d mDesiredPickupLocation) {
        desiredPickupLocation = mDesiredPickupLocation;
    }

    public Pose2d getDesiredPickupLocation() {
        return desiredPickupLocation;
    }

    public void setDesiredScoreLocation(Pose2d mDesiredScoreLocation) {
        desiredScoreLocation = mDesiredScoreLocation;
    }

    public Pose2d getDesiredScoreLocation() {
        return desiredScoreLocation;
    }


}
