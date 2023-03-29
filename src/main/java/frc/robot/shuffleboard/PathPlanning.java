package frc.robot.shuffleboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.StartLocation;
import frc.robot.subsystems.Drivetrain;

public class PathPlanning extends ShuffleboardTabBase{
    private final Drivetrain drivetrain;
    // private final SendableChooser<Alliance> allianceColor;
    private final SendableChooser<ScoreLevel> height;
    private final SendableChooser<PieceType> pieceType;
    private final SendableChooser<Constants.StartLocation> startingLocation;



    public PathPlanning(Drivetrain drivetrain){
        ShuffleboardTab tab = Shuffleboard.getTab("Auto Path");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.drivetrain = drivetrain;
        NetworkTable networkTable = inst.getTable("Shuffleboard/Auto Path");

        // allianceColor = new SendableChooser<>();
        // allianceColor.addOption("Red", Alliance.Red);
        // allianceColor.addOption("Blue", Alliance.Blue);
        // tab.add(allianceColor);

        //level of the first piece
        height = new SendableChooser<>();
        height.addOption("Low", ScoreLevel.LEVEL_1);
        height.addOption("Medium", ScoreLevel.LEVEL_2);
        height.addOption("High", ScoreLevel.LEVEL_3);

        // what were picking up
        pieceType = new SendableChooser<>();
        pieceType.setDefaultOption("Cone", PieceType.CONE);
        pieceType.addOption("Cube", PieceType.CUBE);

        //starting position
        startingLocation = new SendableChooser<>();
        startingLocation.addOption("left", StartLocation.LEFT);
        startingLocation.addOption("middle", StartLocation.MIDDLE);
        startingLocation.addOption("right", StartLocation.RIGHT);

        

        
    }
    public ScoreLevel getScoreLevel(){
        return height.getSelected();
    }

    public PieceType getPieceType(){
        return pieceType.getSelected();
    }

    public StartLocation getStartingLocation(){
        return startingLocation.getSelected();
    }

    public void update(){

    }
    
}
