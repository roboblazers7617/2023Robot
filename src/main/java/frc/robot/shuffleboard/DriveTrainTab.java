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
    DoublePublisher xPosPub;
    DoublePublisher yPosPub;

    Drivetrain drivetrain;

    public void update() {
        // publish the values using the publisher
        rightVelocityPub.set(drivetrain.getRightVelocity());
        leftVelocityPub.set(drivetrain.getLeftVelocity());
        rightDistancePub.set(drivetrain.getRightDistance());
        leftDistancePub.set(drivetrain.getLeftDistance());
        xPosPub.set(drivetrain.getPose2d().getX());
        yPosPub.set(drivetrain.getPose2d().getY());
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

        xPosPub = networkTable.getDoubleTopic("X Cord(m)").publish();

        shuffleboardTabTesting.add("X Cord(m)", 0);

        yPosPub = networkTable.getDoubleTopic("Y Cord(m)").publish();

        shuffleboardTabTesting.add("Y Cord(m)", 0);

    }
}