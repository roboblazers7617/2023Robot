// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainMode;

//if auto things are happining

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
    IntegerSubscriber modeSub;
    private final SendableChooser<String> drivetrainMode = new SendableChooser<>();
    private final SendableChooser<Boolean> debugMode = new SendableChooser<>();

    private Drivetrain drivetrain;
    private DoublePublisher maxSpeedPub;
    private StringPublisher pathPlanningTargetPub;
    private BooleanPublisher debugModePub;

    public DriverStationTab(Drivetrain drivetrain) {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Station");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.drivetrain = drivetrain;

        NetworkTable networkTable = inst.getTable("Shuffleboard/Driver Station");

        drivetrainMode.setDefaultOption("Tank Drive", DrivetrainMode.tankDrive.toString());
        drivetrainMode.addOption("Arcade Drive", DrivetrainMode.arcadeDrive.toString());
        tab.add(drivetrainMode);

        debugMode.setDefaultOption("True", true);
        debugMode.addOption("false", false);
        tab.add(debugMode);
        NetworkTable debugNetworkTable = NetworkTableInstance.getDefault().getTable("debug mode table");
        debugModePub = debugNetworkTable.getBooleanTopic("debug mode").publish();

        maxSpeedPub = networkTable.getDoubleTopic("max SPEED").publish();
        tab.add("max SPEED", 20.0);

        pathPlanningTargetPub = networkTable.getStringTopic("target position for path planning").publish();
        tab.add("target position for path planning", "NA");
    }

    public void update() {
        drivetrain.setDriveTrainMode(DrivetrainMode.valueOf(drivetrainMode.getSelected()));
        maxSpeedPub.set(drivetrain.getCarmax());
        pathPlanningTargetPub.set(drivetrain.getPathPlanningTarget());

        System.out.println(debugMode.getSelected());
        System.out.println("somethings happening");

        debugModePub.set(debugMode.getSelected());

    }
}
