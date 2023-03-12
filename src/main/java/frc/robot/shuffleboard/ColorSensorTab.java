// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.ColorSensor;

/**
 * This is an example of the shuffleboard tab class for a subsystem.
 * 
 * when creating the subsystem tab make sure you extend ShuffleboardTabBase
 */
public class ColorSensorTab extends ShuffleboardTabBase {
    IntegerPublisher redPub;
    IntegerPublisher bluePub;
    IntegerPublisher greenPub;
    IntegerPublisher proximityPub;
    BooleanPublisher momsPub;
    ColorSensor mSensor = new ColorSensor();


    // use IntegerSubscriber to get integers from shuffleboard
    public ColorSensorTab(ColorSensor sensor) {
        // get the main network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // get the table for your individual subsystem
        NetworkTable networkTable = inst.getTable("Shuffleboard/Color Sensor");

        // get the shuffle board tab for the subsystem. the names of the two tabs must
        // be the same
        ShuffleboardTab sensorTab = Shuffleboard.getTab("Color Sensor");

        // create a widget, that is a list
        ShuffleboardLayout data = sensorTab
                .getLayout("Sensor Data", BuiltInLayouts.kList)
                .withSize(2, 4);

        mSensor = sensor;

        // create a publisher in the network table
        redPub = networkTable.getIntegerTopic("Red").publish();
        greenPub = networkTable.getIntegerTopic("Green").publish();
        bluePub = networkTable.getIntegerTopic("Blue").publish();
        proximityPub = networkTable.getIntegerTopic("Proximity").publish();
        momsPub = networkTable.getBooleanTopic("Testing Threshold").publish();

        // add the network table to shuffleboard, the name must be the same, the default
        // value does not matter.
        data.add("Red", 0);
        data.add("Green", 0);

        data.add("Blue", 0);

        data.add("Proximity", 0);
        data.add("Testing Threshold", false);

    }

    public void update() {
        // publish the values using the publisher
        redPub.set(mSensor.getRed());
        bluePub.set(mSensor.getBlue());
        greenPub.set(mSensor.getGreen());
        proximityPub.set(mSensor.getProximity());
        momsPub.set(mSensor.getProximity()>600);

    }
}
