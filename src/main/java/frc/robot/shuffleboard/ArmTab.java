// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;


import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

/**
 * This is the arm shuffleboard tab class for a subsystem.
 * 
 * when creating the subsystem tab make sure you extend ShuffleboardTabBase
 */
public class ArmTab extends ShuffleboardTabBase {
    DoublePublisher anglePub;
    DoublePublisher motorTempPub;
    StringPublisher pistonPub;
    Arm mArm;

    // use IntegerSubscriber to get integers from shuffleboard
    public ArmTab(Arm arm) {
        // get the main network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // get the table for your individual subsystem
        NetworkTable networkTable = inst.getTable("Shuffleboard/Arm");

        // get the shuffle board tab for the subsystem. the names of the two tabs must
        // be the same
        ShuffleboardTab shuffleboardTabTesting = Shuffleboard.getTab("Arm");

        // create a widget, that is a list
        ShuffleboardLayout widget = shuffleboardTabTesting
                .getLayout("Positional Info", BuiltInLayouts.kList)
                .withSize(2, 2);

        mArm = arm;

        // create a publisher in the network table
        anglePub = networkTable.getDoubleTopic("Angle (Degrees)").publish();
        pistonPub = networkTable.getStringTopic("Piston state").publish();
        motorTempPub = networkTable.getDoubleTopic("Motor Temp").publish();

        // add the network table to shuffleboard, the name must be the same, the default
        // value does not matter.
        widget.add("Angle (Degrees)", 0);
        widget.add("Piston State", "ERROR 404: Position does not exist.");

        shuffleboardTabTesting.add("Disable Compressor", new InstantCommand(() ->mArm.enableCompressor(false)));
        shuffleboardTabTesting.add("Enable Compressor", new InstantCommand(() ->mArm.enableCompressor(true)));
    }

    public void update() {
        // publish the values using the publisher
        anglePub.set(mArm.getShoulderAngle());
        pistonPub.set(mArm.getSuperstructureState().toString());
        motorTempPub.set((mArm.getShoulderMotorTemp()*(9.0/5.0)+32.0));

    }
}
