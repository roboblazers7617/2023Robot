// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase{
    IntegerSubscriber modeSub;
    private final SendableChooser<String> drivetrainMode = new SendableChooser<>();



    public DriverStationTab(){
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Station");

        // tab.add("Drive Train Mode", 1);

        // modeSub = networkTable.getIntegerTopic("Drive Train Mode").subscribe(1);
        drivetrainMode.setDefaultOption("Tank Drive", Constants.MECANUM_DRIVE_STRING);
        drivetrainMode.addOption("Mecanum Drive", Constants.TANK_DRIVE_STRING);
        tab.add(drivetrainMode);

    }




    public void update(){
        System.out.println("some text " + drivetrainMode.getSelected());
    }
}
