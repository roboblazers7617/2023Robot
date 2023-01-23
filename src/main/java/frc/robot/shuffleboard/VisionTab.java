// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/** This is an example of the shuffleboard tab class for a subsystem. 
 * 
 * when creating the subsystem tab make sure you extend ShuffleboardTabBase*/
public class VisionTab extends ShuffleboardTabBase{
    DoublePublisher angleToTagPub;
    DoublePublisher distanceFromTagPub;
    StringPublisher bestTagIdPub;
    Vision mVision;
    Drivetrain mDrivetrain;

    // use IntegerSubscriber to get integers from shuffleboard
    public VisionTab(Vision vision, Drivetrain drivetrain){
        //get the main network table
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        //get the table for your individual subsystem
        NetworkTable networkTable = inst.getTable("Shuffleboard/Vision");
        

        //get the shuffle board tab for the subsystem. the names of the two tabs must be the same
        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
        
        mVision = vision;
        mDrivetrain = drivetrain;

        //create a publisher in the network table
        angleToTagPub = networkTable.getDoubleTopic("Angle To Tag").publish();
        distanceFromTagPub = networkTable.getDoubleTopic("Distance From Tag").publish();
        bestTagIdPub = networkTable.getStringTopic("Best Tag ID").publish();
        
        //add the network table to shuffleboard, the name must be the same, the default value does not matter. 
        visionTab.add("Angle To Tag", 0);
        visionTab.add("Distance From Tag", 0);
        visionTab.add("Best Tab ID", "No tag found.");
        
    }
    public void update(){
        //publish the values using the publisher
        angleToTagPub.set(mVision.getBestTagYaw());
        distanceFromTagPub.set(mVision.getBestTagDistance());
        if(mVision.getBestTagId() != -1)
            bestTagIdPub.set(String.valueOf(mVision.getBestTagId()));
        else
            bestTagIdPub.set("No tag found.");
        
    }
}
