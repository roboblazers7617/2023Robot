// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;

/**
 * This is an example of the shuffleboard tab class for a subsystem.
 * 
 * when creating the subsystem tab make sure you extend ShuffleboardTabBase
 */
public class IntakeTab extends ShuffleboardTabBase {
    DoublePublisher anglePub;
    Intake intake;

    // use IntegerSubscriber to get integers from shuffleboard
    public IntakeTab(Intake intake) {
        // get the main network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // get the table for your individual subsystem
        NetworkTable networkTable = inst.getTable("Shuffleboard/intake");

        // get the shuffle board tab for the subsystem. the names of the two tabs must
        // be the same
        ShuffleboardTab shuffleboardTabTesting = Shuffleboard.getTab("intake");

        this.intake = intake;

        // create a publisher in the network table
        anglePub = networkTable.getDoubleTopic("Wrist Angle").publish();


    }

    public void update() {
        // publish the values using the publisher
        anglePub.set(intake.getWristAngle());

    }
}
