package frc.robot.shuffleboard;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Intake;

public class IntakeTab extends ShuffleboardTabBase {
    DoublePublisher wristAnglePublisher;
    DoublePublisher wristSpeedPublisher;
    DoublePublisher intakeSpeedPublisher;
    DoublePublisher wristTempPublisher;
    DoublePublisher intakeTempPublisher;
    BooleanPublisher isIntakeStoredPublisher;
    BooleanPublisher isHoldingCubePublisher;
    DoublePublisher wristEncoderAnglePublisher;
    DoublePublisher intakeCurrentPublisher;
    Intake intake;
    public IntakeTab (Intake subsystem){
        intake = subsystem;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable networkTable = inst.getTable("Shuffleboard/Intake");
        Shuffleboard.getTab("Intake");

        
        isHoldingCubePublisher = networkTable.getBooleanTopic("Have Cube?").publish();
        intakeCurrentPublisher = networkTable.getDoubleTopic("Intake Current").publish();
        intakeTempPublisher = networkTable.getDoubleTopic("Intake Temp").publish();
        intakeSpeedPublisher = networkTable.getDoubleTopic("Intake Speed").publish();
        //shuffleboardTabTesting.add(new intake.moveToPositionCommand(IntakeConstants.WristPosition.FloorConePickup));
        
    }

    public void update() {
        intakeSpeedPublisher.set(intake.getIntakeSpeed());
        intakeTempPublisher.set((intake.getMotorTemperature()*(9.0/5.0)+32.0));
        intakeCurrentPublisher.set(intake.getCurent());

    }
    
}
