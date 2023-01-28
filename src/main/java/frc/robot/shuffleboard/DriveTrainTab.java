package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;

public class DriveTrainTab extends ShuffleboardTabBase {
    DoublePublisher rightVelocityPub;
    DoublePublisher leftVelocityPub;
    DoublePublisher leftDistancePub;
    DoublePublisher rightDistancePub;
    Drivetrain drivetrain;

    public void update() {
        // publish the values using the publisher
        rightVelocityPub.set(drivetrain.getRigthVelocity());
        leftVelocityPub.set(drivetrain.getLeftVelocity());
        rightDistancePub.set(drivetrain.getRightDistance());
        leftDistancePub.set(drivetrain.getLeftDistance());
    }

    public DriveTrainTab(Drivetrain subsystem) {

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable networkTable = inst.getTable("Shuffleboard/drivetrain");

        ShuffleboardTab shuffleboardTabTesting = Shuffleboard.getTab("drivetrain");

        drivetrain = subsystem;

        rightVelocityPub = networkTable.getDoubleTopic("right speed").publish();

        shuffleboardTabTesting.add("right speed", 3);

        leftVelocityPub = networkTable.getDoubleTopic("left speed").publish();

        shuffleboardTabTesting.add("left speed", 3);

        rightDistancePub = networkTable.getDoubleTopic("right position").publish();

        shuffleboardTabTesting.add("right position", 3);

        leftDistancePub = networkTable.getDoubleTopic("left position").publish();

        shuffleboardTabTesting.add("left position", 3);

    }
}