// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CenterRelativeTag;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeDown;
//import frc.robot.commands.IntakeDown;
import frc.robot.commands.ScoreGridSelection;
import frc.robot.commands.Drivetrain.DriveToScoreGrid;
import frc.robot.commands.TurnToTag;
import frc.robot.commands.centerAndDistanceAlign;
import frc.robot.shuffleboard.ColorSensorTab;
import frc.robot.shuffleboard.DriveTrainTab;
import frc.robot.shuffleboard.DriverStationTab;
import frc.robot.shuffleboard.ExampleSubsystemTab;
import frc.robot.shuffleboard.IntakeTab;
import frc.robot.shuffleboard.ShuffleboardInfo;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.shuffleboard.VisionTab;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Vision vision = new Vision();
  private final Drivetrain drivetrain = new Drivetrain(vision);
  private final Leds leds = new Leds();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);
  private Pose2d targetPose = new Pose2d(new Translation2d(Units.inchesToMeters(40.45+36),Units.inchesToMeters(42.19)),
  new Rotation2d(180));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureDriverBindings();
    configureOperatorBindings();
    // create shuffleboardinfo.java
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(m_driverController.getLeftY(),
        m_driverController.getRightX(), m_driverController.getRightY(), true), drivetrain));
    // arm.setDefaultCommand(new RunCommand(() -> arm.setShoulderSpeed(m_operatorController.getLeftY()), arm));
    
    // intake.setDefaultCommand(new RunCommand(()->intake.setWristSpeed( m_operatorController.getRightY()),intake));
    ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
    // YOUR CODE HERE | | |
    // \/ \/ \/
    tabs.add(new DriverStationTab(drivetrain));
    tabs.add(new ExampleSubsystemTab(m_exampleSubsystem));
    tabs.add(new VisionTab(vision, drivetrain));
    tabs.add(new DriveTrainTab(drivetrain));
    tabs.add(new ColorSensorTab(new ColorSensor()));
    // STOP HERE OR DIE

    ShuffleboardInfo shuffleboardInfo = ShuffleboardInfo.getInstance();
    shuffleboardInfo.addTabs(tabs);

    // create some tabs

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriverBindings() {
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // m_driverController.a().whileTrue(new TurnToTag(vision, drivetrain));
    //m_driverController.a().whileTrue(new RunCommand(PickPathWork(drivetrain, ()-> drivetrain.getPose2d().getX(), ()-> drivetrain.getPose2d().getY(),()-> drivetrain.getPose2d().getRotation().getDegrees())));
    // m_driverController.rightTrigger().whileTrue(new centerAndDistanceAlign(vision, drivetrain, 1));
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
    /* 
    //this code calls the score grid selection command for the correct input
    /*m_driverController.x()
        .and(m_driverController.povLeft())
        .onTrue(new ScoreGridSelection(drivetrain, 0, 0));
    m_driverController.y()
        .and(m_driverController.povLeft())
        .onTrue(new ScoreGridSelection(drivetrain, 0, 1));
    m_driverController.b()
        .and(m_driverController.povLeft())
        .onTrue(new ScoreGridSelection(drivetrain, 0, 2));
    m_driverController.x()
        .and(m_driverController.povUp())
        .onTrue(new ScoreGridSelection(drivetrain, 1, 0));
    m_driverController.y()
        .and(m_driverController.povUp())
        .onTrue(new ScoreGridSelection(drivetrain, 1, 1));
    m_driverController.b()
        .and(m_driverController.povUp())
        .onTrue(new ScoreGridSelection(drivetrain, 1, 2));
    m_driverController.x()
        .and(m_driverController.povRight())
        .onTrue(new ScoreGridSelection(drivetrain, 2, 0));
    m_driverController.y()
        .and(m_driverController.povRight())
        .onTrue(new ScoreGridSelection(drivetrain, 2, 1));
    m_driverController.b()
        .and(m_driverController.povRight())
        .onTrue(new ScoreGridSelection(drivetrain, 2, 2));*/
    m_driverController.povLeft().whileTrue(
    new InstantCommand(()-> setTargetPose(
        new Pose2d(new Translation2d(Units.inchesToMeters(40.45+38),Units.inchesToMeters(108.19)),
       new Rotation2d(Units.degreesToRadians(0))))));
    m_driverController.povUp().onTrue(new InstantCommand(()-> setTargetPose(
        new Pose2d(new Translation2d(Units.inchesToMeters(40.45+38),Units.inchesToMeters(42.19)),
        new Rotation2d((Units.degreesToRadians(0)))))));
    m_driverController.povRight().onTrue(new InstantCommand(()-> setTargetPose(
        new Pose2d(new Translation2d(Units.inchesToMeters(40.45+38),Units.inchesToMeters(42.19)),
        new Rotation2d(Units.degreesToRadians(0))))));

    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(m_driverController.getLeftY(),
        m_driverController.getRightX(), m_driverController.getRightY(), false), drivetrain));
    // TODO: Imptement whatever system to select node to go to
    m_driverController.leftTrigger().whileTrue(new DriveToScoreGrid(drivetrain, 
        ()-> m_driverController.getLeftY(), 
        ()-> m_driverController.getRightY(), 
        ()-> m_driverController.getRightX(), 
        ()->getTargetPose(),
        Alliance.Blue));

        // (new Translation2d((Units.inchesToMeters(20)),(Units.inchesToMeters(155)))));
    // m_driverController.rightTrigger().whileTrue(null/*TODO:  go to double substation*/);
    // m_driverController.rightTrigger().and(m_driverController.leftTrigger()).whileTrue(null /*TODO  go to single substation*/);
    m_driverController.rightTrigger().onTrue(new InstantCommand(()-> drivetrain.resetEncoders()).andThen(new InstantCommand(()-> drivetrain.resetOdometry(new Pose2d(0,0,new Rotation2d(0))))).andThen(new InstantCommand (()-> drivetrain.zeroHeading())));

    m_driverController.rightBumper().onTrue(new InstantCommand(() -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.SLOW_SPEED)));
    m_driverController.rightBumper().onFalse(new InstantCommand(() -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED)));

    m_driverController.leftBumper().onTrue(new InstantCommand(() -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.FAST_SPEED)));
    m_driverController.leftBumper().onFalse(new InstantCommand(() -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED)));

    m_driverController.b().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d())));
    //TODO: Adressable LEDS
    
  }

  private void configureOperatorBindings() {
    //TODO: Arm.setDefaultCommand(use L joy to move);
    //TODO Intake.setDefaultCommand(use R joy to move);
//    m_operatorController.leftBumper().whileTrue(null/*TODO: Pick up cube double substation */);
//    m_operatorController.rightBumper().whileTrue(null/*TODO: Pick up cone double substation */);
//    m_operatorController.leftBumper().whileTrue(null/*TODO: Pick up cube floor */);
//    m_operatorController.rightBumper().whileTrue(null/*TODO: Pick up cone floor */);

//    m_operatorController.a().onTrue(null /*TODO: Toggle Arm pnuematic */);
//    m_operatorController.b().whileTrue(null /*TODO: move arm to directed score level */);
//    m_operatorController.y().whileTrue(null /*TODO: Spin intake to pick-up cube/ spit cone */);
//    m_operatorController.x().whileTrue(null /*TODO: Spin intake to spit cube/ pick-up cone */);

//    m_operatorController.povLeft(/*TODO: Store Arm */);

//    m_operatorController.povDown().onTrue(null /*TODO: set to hybrid node position */);
//    m_operatorController.povLeft().onTrue(null /*TODO: set to level 2 node position */);
//    m_operatorController.povDown().onTrue(null /*TODO: set to level 3 node position */);



  }
  public void setTargetPose(Pose2d targetPose) {
    this.targetPose = targetPose;
    }
  public Pose2d getTargetPose() {
    return targetPose;
  }
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */   
  public Command getAutonomousCommand() {
    drivetrain.setBrakeMode(IdleMode.kCoast);
    return pickAutonomousCommand("blue far 2 ball").andThen(() -> drivetrain.setBrakeMode(IdleMode.kBrake));
    }
  public Command pickAutonomousCommand(String pathName) {   
     
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("intakeDown", new IntakeDown());

    PathPlannerTrajectory test_path = PathPlanner.loadPath(
        pathName, new PathConstraints(DrivetrainConstants.MAX_AUTO_VELOCITY, 
        DrivetrainConstants.MAX_AUTO_ACCELERATION),true);
//   drivetrain.resetOdometry(test_path.getInitialPose());
//    PPRamseteCommand returnCommand = new PPRamseteCommand(
//       test_path, 
//       drivetrain::getPose2d, 
//       new RamseteController(), 
//       new SimpleMotorFeedforward(DrivetrainConstants.KS_LIN, DrivetrainConstants.KV),
//       drivetrain.getKinematics(),
//       drivetrain::getWheelSpeeds,
//       new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN),
//       new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN),
//       drivetrain::tankDriveVolts,
//       false,
//       drivetrain);
//       return returnCommand;
       RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
        drivetrain::getPose2d, // Pose2d supplier
        drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        new RamseteController(2.0,0.7),
        drivetrain.getKinematics(),
        new SimpleMotorFeedforward(DrivetrainConstants.KS_LIN, DrivetrainConstants.KV),
        ()->drivetrain.getWheelSpeeds(), // WheelSpeeds supplier
        new PIDConstants(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN),
        // PID constants to correct for rotation error (used to create the rotation controller)
        drivetrain::tankDriveVolts, // Module states consumer used to output to the drive subsystem
        eventMap,
        false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );
    

  return autoBuilder.fullAuto(test_path);
  }
  }

