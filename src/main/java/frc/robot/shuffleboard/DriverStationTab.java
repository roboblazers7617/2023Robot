// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;


/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase{
    IntegerSubscriber modeSub;
    private final SendableChooser<String> drivetrainMode = new SendableChooser<>();
    private Drivetrain drivetrain;
    private DoublePublisher maxSpeedPub;

    public DriverStationTab(Drivetrain drivetrain){
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Station");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.drivetrain = drivetrain;
        // tab.add("Drive Train Mode", 1);
        NetworkTable networkTable = inst.getTable("Shuffleboard/Driver Station");

        // modeSub = networkTable.getIntegerTopic("Drive Train Mode").subscribe(1);
        drivetrainMode.setDefaultOption("Tank Drive", DrivetrainConstants.DrivetrainMode.tankDrive.toString());
        drivetrainMode.addOption("Arcade Drive", DrivetrainConstants.DrivetrainMode.arcadeDrive.toString());
        tab.add(drivetrainMode);

        maxSpeedPub = networkTable.getDoubleTopic("max SPEED").publish();
        tab.add("max SPEED", 20.0);
    }




    public void update(){
        drivetrain.setDriveTrainMode(DrivetrainConstants.DrivetrainMode.valueOf(drivetrainMode.getSelected()));
        maxSpeedPub.set(drivetrain.getCarmax());
}
}
