package frc.robot.shuffleboard;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class IntakeTab extends ShuffleboardTabBase {
    Wrist wrist;
    DoublePublisher wristAnglePublisher;
    DoublePublisher wristSpeedPublisher;
    DoublePublisher intakeSpeedPublisher;
    DoublePublisher wristTempPublisher;
    BooleanPublisher isIntakeStoredPublisher;
    BooleanPublisher isHoldingCubePublisher;
    DoublePublisher wristEncoderAnglePublisher;
    Intake intake;
    public IntakeTab (Intake subsystem, Wrist wrist){
        intake = subsystem;
        this.wrist = wrist;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable networkTable = inst.getTable("Shuffleboard/Intake");
        ShuffleboardTab shuffleboardTabTesting = Shuffleboard.getTab("Intake");
        wristAnglePublisher = networkTable.getDoubleTopic("The angle of the wrist").publish();
        wristTempPublisher = networkTable.getDoubleTopic("Wrist Temp").publish();
        wristSpeedPublisher = networkTable.getDoubleTopic("The speed of the wrist").publish();
        intakeSpeedPublisher = networkTable.getDoubleTopic("The speed of the intake").publish();
        isIntakeStoredPublisher = networkTable.getBooleanTopic("Is the intake in the stored position?").publish();
        isHoldingCubePublisher = networkTable.getBooleanTopic("Is the robot holding a cube?").publish();
        wristEncoderAnglePublisher = networkTable.getDoubleTopic("Wrist Encoder Angle").publish();
        
        //shuffleboardTabTesting.add(new intake.moveToPositionCommand(IntakeConstants.WristPosition.FloorConePickup));
        
    }

    public void update() {
        wristAnglePublisher.set(wrist.getWristPosition());
        wristSpeedPublisher.set(wrist.getWristVelocity());
        intakeSpeedPublisher.set(intake.getIntakeSpeed());
        isIntakeStoredPublisher.set(intake.isStored());
        isHoldingCubePublisher.set(intake.isHoldingGamePiece());
       // wristEncoderAnglePublisher.set(intake.getEncoderAngle());
        wristTempPublisher.set((wrist.getWristMotorTemp()*(9.0/5.0)+32.0));

    }
    
}
