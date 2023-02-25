// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shuffleboard;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.FieldPositions;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.DrivetrainMode;
import frc.robot.FieldPositions.FieldLocation;
import frc.robot.subsystems.Drivetrain;

//if auto things are happining

/** Add your docs here. */
public class DriverStationTab extends ShuffleboardTabBase {
    IntegerSubscriber modeSub;
    private final SendableChooser<String> drivetrainMode = new SendableChooser<>();
    private final SendableChooser<Boolean> debugMode = new SendableChooser<>();
    private final SendableChooser<FieldPositions.FieldLocation> targetNode = new SendableChooser<>();


    private Drivetrain drivetrain;
    private DoublePublisher maxSpeedPub;
    private StringPublisher pathPlanningTargetPub;
    private BooleanPublisher debugModePub;

    private UsbCamera camera;

    public DriverStationTab(Drivetrain drivetrain) {
        //tab and network table
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Station");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.drivetrain = drivetrain;
        NetworkTable networkTable = inst.getTable("Shuffleboard/Driver Station");

        //drive mode
        drivetrainMode.setDefaultOption("Tank Drive", DrivetrainConstants.DrivetrainMode.tankDrive.toString());
        drivetrainMode.addOption("Arcade Drive", DrivetrainMode.arcadeDrive.toString());
        drivetrainMode.addOption("Curvature Drive", DrivetrainMode.curvatureDrive.toString());
        tab.add(drivetrainMode);

        //debug mode
        debugMode.setDefaultOption("True", true);
        debugMode.addOption("false", false);
        tab.add(debugMode);
        NetworkTable debugNetworkTable = NetworkTableInstance.getDefault().getTable("debug mode table");
        debugModePub = debugNetworkTable.getBooleanTopic("debug mode").publish();

        //path planning target
        targetNode.setDefaultOption("Node 1", FieldLocation.NODE1);
        targetNode.addOption("Node 2", FieldLocation.NODE2);
        targetNode.addOption("Node 3", FieldLocation.NODE3);
        targetNode.addOption("Node 4", FieldLocation.NODE4);
        targetNode.addOption("Node 5", FieldLocation.NODE5);
        targetNode.addOption("Node 6", FieldLocation.NODE6);
        targetNode.addOption("Node 7", FieldLocation.NODE7);
        targetNode.addOption("Node 8", FieldLocation.NODE8);
        targetNode.addOption("Node 9", FieldLocation.NODE9);
        tab.add("field target", targetNode);
        


        //current speed modifier
        maxSpeedPub = networkTable.getDoubleTopic("max SPEED").publish();
        tab.add("max SPEED", 20.0);

        //path planning target
        pathPlanningTargetPub = networkTable.getStringTopic("target position for path planning").publish();
        tab.add("target position for path planning", "NA");

        camera = CameraServer.startAutomaticCapture();
        if (camera.isConnected()) {
            camera.setResolution(480, 320);
            camera.setFPS(10);
        }
    }

    public void update() {
        drivetrain.setDriveTrainMode(DrivetrainMode.valueOf(drivetrainMode.getSelected()));
        maxSpeedPub.set(drivetrain.getCarmax());
        pathPlanningTargetPub.set(drivetrain.getTargetPose());

        debugModePub.set(debugMode.getSelected());
        drivetrain.setTargetNode(targetNode.getSelected());

    }
}
