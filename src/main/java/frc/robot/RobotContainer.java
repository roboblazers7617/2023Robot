// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CenterRelativeTag;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ScoreGridSelection;
import frc.robot.commands.ToggleArmPnuematics;
import frc.robot.shuffleboard.DriveTrainTab;
import frc.robot.shuffleboard.DriverStationTab;
import frc.robot.shuffleboard.ExampleSubsystemTab;
import frc.robot.shuffleboard.ShuffleboardInfo;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.shuffleboard.VisionTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pnuematics;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;

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
  private final Pnuematics pnuematics = new Pnuematics();
  private final Arm arm = new Arm(pnuematics);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureDriverBindings();
    configureOperatorBindings();
    // create shuffleboardinfo.java
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(m_driverController.getLeftY(),
        m_driverController.getRightX(), m_driverController.getRightY()), drivetrain));
    arm.setDefaultCommand(new RunCommand(() -> arm.setShoulderSpeed(m_operatorController.getLeftX()), arm));
    
    ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
    // YOUR CODE HERE | | |
    // \/ \/ \/
    tabs.add(new ExampleSubsystemTab(m_exampleSubsystem));
    tabs.add(new DriverStationTab(drivetrain));
    tabs.add(new VisionTab(vision, drivetrain));
    tabs.add(new DriveTrainTab(drivetrain));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    Trigger leftTop = m_driverController.leftBumper();
    leftTop.onTrue(new InstantCommand(() -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.SLOW_SPEED)))
        .onFalse(new InstantCommand(() -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.MAX_SPEED)));

    Trigger rightTop = m_driverController.rightBumper();
    rightTop.onTrue(new InstantCommand(() -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.FAST_SPEED)))
        .onFalse(new InstantCommand(() -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.MAX_SPEED)));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // m_driverController.a().whileTrue(new TurnToTag(vision, drivetrain));
    // m_driverController.x().whileTrue(new DriveToTag(vision, drivetrain, 1));
    // m_driverController.rightTrigger().whileTrue(new centerAndDistanceAlign(vision, drivetrain, 1));
        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.

    //this code calls the score grid selection command for the correct input
    m_driverController.x()
        .and(m_driverController.povLeft())
        .onTrue(new ScoreGridSelection(0, 0));
    m_driverController.y()
        .and(m_driverController.povLeft())
        .onTrue(new ScoreGridSelection(0, 1));
    m_driverController.b()
        .and(m_driverController.povLeft())
        .onTrue(new ScoreGridSelection(0, 2));
    m_driverController.x()
        .and(m_driverController.povUp())
        .onTrue(new ScoreGridSelection(1, 0));
    m_driverController.y()
        .and(m_driverController.povUp())
        .onTrue(new ScoreGridSelection(1, 1));
    m_driverController.b()
        .and(m_driverController.povUp())
        .onTrue(new ScoreGridSelection(1, 2));
    m_driverController.x()
        .and(m_driverController.povRight())
        .onTrue(new ScoreGridSelection(2, 0));
    m_driverController.y()
        .and(m_driverController.povRight())
        .onTrue(new ScoreGridSelection(2, 1));
    m_driverController.b()
        .and(m_driverController.povRight())
        .onTrue(new ScoreGridSelection(2, 2));

    m_driverController.leftTrigger().whileTrue(new CenterRelativeTag(vision,drivetrain,0.25));
  }

  private void configureOperatorBindings() {

    // set height to high
    m_operatorController.rightBumper()
        .and(m_operatorController.y())
        .whileTrue(new InstantCommand(() -> m_exampleSubsystem.yPressed()));

    m_operatorController.rightBumper()
        .and(m_operatorController.povDown())
        .whileTrue(new InstantCommand(() -> m_exampleSubsystem.povDownPressed()));

    m_operatorController.a().whileTrue(new ToggleArmPnuematics(arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
