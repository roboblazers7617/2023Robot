// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.ExampleSubsystem;

/** This is an example of the shuffleboard tab class for a subsystem. 
 * 
 * when creating the subsystem tab make sure you extend ShuffleboardTabBase*/
public class ExampleSubsystemTab extends ShuffleboardTabBase{
    IntegerPublisher xPub;
    ExampleSubsystem exampleSubsystem;

    public ExampleSubsystemTab(ExampleSubsystem subsystem){
        //get the main network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        //get the table for your individual subsystem
        NetworkTable networkTable = inst.getTable("Shuffleboard/example subsystem");
        

        //get the shuffle board tab for the subsystem. the names of the two tabs must be the same
        ShuffleboardTab shuffleboardTabTesting = Shuffleboard.getTab("example subsystem");
        
        exampleSubsystem = subsystem;

        //create a publisher in the network table
        xPub = networkTable.getIntegerTopic("x").publish();
        
        //add the network table to shuffleboard, the name must be the same, the default value does not matter. 
        shuffleboardTabTesting.add("x", 3);
        
    }
    public void update(){
        //publish the values using the publisher
        xPub.set(exampleSubsystem.getData());
        
    }
}
