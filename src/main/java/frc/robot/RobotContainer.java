// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.DrivetrainConstants.AutoPath;
import frc.robot.Constants.ArmConstants.StateConstants.GenericPosition;
import frc.robot.Constants.ArmConstants.StateConstants.StatePosition;
import frc.robot.commands.ArmStuff.SimpleScore;
import frc.robot.commands.ArmStuff.Stow;
import frc.robot.shuffleboard.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.States.*;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.States.AutoBalanceState;
import frc.robot.subsystems.Drivetrain.States.FaceScoreLocationState;
import frc.robot.subsystems.Drivetrain.States.FollowPathState;
import frc.robot.subsystems.Drivetrain.States.ResetOdometryState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.States.IntakePiece;
import frc.robot.subsystems.Intake.States.OutakePiece;

import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        private final Vision vision = new Vision();
        private final Drivetrain drivetrain = new Drivetrain(vision);
        // private final Leds leds = new Leds();
        private final Pnuematics pnuematics = new Pnuematics();
        private final Intake intake = new Intake();
        private final Arm arm = new Arm(pnuematics);
        private final Leds leds = new Leds();

        private final CommandXboxController m_driverController = new CommandXboxController(
                        OperatorConstants.DRIVER_CONTROLLER_PORT);

        private final CommandXboxController m_operatorController = new CommandXboxController(
                        OperatorConstants.OPERATOR_CONTROLLER_PORT);

        private Pose2d targetPose = new Pose2d(
                        new Translation2d(Units.inchesToMeters(40.45 + 36), Units.inchesToMeters(42.19)),
                        new Rotation2d(180));

        private PieceType selectedPiece;

        private final DriverStationTab driverStationTab;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                setSelectedPiece(PieceType.Cone);
                // Configure the trigger bindings
                configureDriverBindings();
                configureOperatorBindings();
                // create shuffleboardinfo.java
                
                ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
                // YOUR CODE HERE | | |
                // \/ \/ \/
                driverStationTab = new DriverStationTab(drivetrain, intake);
                tabs.add(driverStationTab);
                tabs.add(new VisionTab(vision, drivetrain));

                tabs.add(new DriveTrainTab(drivetrain));
                // tabs.add(new ColorSensorTab(new ColorSensor()));
                tabs.add(new IntakeTab(intake));
                tabs.add(new ArmTab(arm));
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

                drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(m_driverController.getLeftY(),
                m_driverController.getRightX(), m_driverController.getRightY(),
                () -> isRightTriggerPressed()),
                drivetrain));

                m_driverController.rightBumper()
                                .onTrue(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.SLOW_SPEED)));
                m_driverController.rightBumper()
                                .onFalse(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED)));

                m_driverController.rightTrigger()
                                .onTrue(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.REALLY_SLOW_SPEED)));
                m_driverController.rightTrigger()
                                .onFalse(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED)));
                //
                m_driverController.leftBumper()
                                .onTrue(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.FAST_SPEED)));
                m_driverController.leftBumper()
                                .onFalse(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED)));

                // m_driverController.b().onTrue(new InstantCommand(() ->
                // drivetrain.resetOdometry(new Pose2d())));
                m_driverController.a().whileTrue(new AutoBalanceState(drivetrain));
                m_driverController.x().onTrue(new InstantCommand(() -> drivetrain.toggleBrakeMode()));
                /*
                 * Trigger doubleSubstationAlign = new Trigger(vision::inRangeOfDoubleStation);
                 * m_driverController.leftTrigger()
                 * .whileTrue(new IntakeAtDouble(vision, drivetrain, arm, wrist, intake,
                 * () -> getSelectedPiece(), () -> m_driverController.getLeftY(),
                 * () -> m_driverController.getRightY(),
                 * () -> m_driverController.getRightX(), () -> isRightTriggerPressed()))
                 * .onFalse(new InstantCommand(
                 * () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED)));
                 */
        }

        private void configureOperatorBindings() {
                arm.setDefaultCommand(new SetVelocitiesState(arm, m_operatorController::getRightY, m_operatorController::getLeftY));

                m_operatorController.leftBumper()
                                .onTrue(arm.changeState(() -> getSelectedPiece(), GenericPosition.DoublePickup, true));
                m_operatorController.leftTrigger()
                                .onTrue(arm.changeState(() -> getSelectedPiece(), GenericPosition.FloorPickup, true));

                m_operatorController.rightBumper()
                                .onTrue(Commands.runOnce(() -> setSelectedPiece(PieceType.Cone)));
                m_operatorController.rightTrigger()
                                .onTrue(Commands.runOnce(() -> setSelectedPiece(PieceType.Cone)));

                m_operatorController.y().whileTrue(new IntakePiece(intake, () -> getSelectedPiece()));

                m_operatorController.x()
                                .whileTrue(new OutakePiece(intake, () -> getSelectedPiece()));

                m_operatorController.a().onTrue(new ToggleArmPnuematics(arm));


                m_operatorController.povLeft()
                .onTrue(arm.changeState(() -> getSelectedPiece(), GenericPosition.Stow, true));
                m_operatorController.povDown().onTrue(
                        arm.changeState(() -> getSelectedPiece(), GenericPosition.Level1, true));
                m_operatorController.povRight().onTrue(
                        arm.changeState(() -> getSelectedPiece(), GenericPosition.Level2, true));
                m_operatorController.povUp().onTrue(
                        arm.changeState(() -> getSelectedPiece(), GenericPosition.Level3, true));

              //  m_operatorController.start().onTrue(new InstantCommand(arm::removeBounds, arm)).onFalse(new InstantCommand(arm::addBounds, arm)).onFalse(new InstantCommand(arm::resetEncoders, arm));
               // m_operatorController.back().onTrue(new InstantCommand(arm::removeBounds, arm)).onFalse(new InstantCommand(arm::addBounds, arm)).onFalse(new InstantCommand(arm::resetEncoders, arm));
        }

        public void stow() {
                arm.setPosition(StatePosition.Stow);
        }

        public void setTargetPose(Pose2d targetPose) {
                this.targetPose = targetPose;
        }

        public Pose2d getTargetPose() {
                return targetPose;
        }

        public PieceType getSelectedPiece() {
                return selectedPiece;
        }

        public void setSelectedPiece(PieceType piece) {
                if (piece == PieceType.Cone) {
                //        leds.orange();
                } else {
                //        leds.purple();
                }
                selectedPiece = piece;
        }

        public void turnOnMechanismBrakes(Boolean isBraked) {
                arm.turnOnBrakes(isBraked);
        }

        public void turnOnBrakesDrivetrain(Boolean isTrue) {
                drivetrain.turnOnBrakes(isTrue);
        }

        private boolean isRightTriggerPressed() {
                return m_driverController.getRightTriggerAxis() > 0.5;
        };

        private double setAutoBalanceAcceleration() {
                if (driverStationTab.getAutoPath().autoBalance() == true) {
                        return 1.5;
                } else {
                        return 2.5;
                }
        }

        private double setAutoBalanceVelocity() {
                if (driverStationTab.getAutoPath().autoBalance() == true) {
                        return 1.5;
                } else {
                        return 2.5;
                }
        }

        public Command getPathPlannerCommand() {
                PathPlannerTrajectory path = PathPlanner.loadPath(driverStationTab.getAutoPath().pathname(),
                                new PathConstraints(setAutoBalanceVelocity(),
                                                setAutoBalanceAcceleration()),
                                driverStationTab.getAutoPath().isReverse());

                return new ResetOdometryState(drivetrain, path).andThen(new FollowPathState(drivetrain, path));
        }

        public Command getReturnPathPlannerCommand() {
                PathPlannerTrajectory path = PathPlanner.loadPath(driverStationTab.getAutoPath().returnpathname(),
                                new PathConstraints(setAutoBalanceVelocity(),
                                                setAutoBalanceAcceleration()),
                                false);

                                return new FollowPathState(drivetrain, path);
        }

        public Command getPickupPathPlannerCommand() {
                PathPlannerTrajectory path = PathPlanner.loadPath(driverStationTab.getAutoPath().pickuppathname(),
                                new PathConstraints(1,
                                                1),
                                false);
                return new FollowPathState(drivetrain, path);
        }

        public SequentialCommandGroup SimpleAuto(AutoPath AutoPath) {
                SequentialCommandGroup auto = new SequentialCommandGroup();
                if (driverStationTab.getAutoPath().scoring()) {
                        auto.addCommands(new SimpleScore(arm, intake, () -> AutoPath.selectedPiece(), () -> AutoPath.scoreLevelFirst(), false));
                }
                auto.addCommands(new InstantCommand(() -> turnOnBrakesDrivetrain(false)),
                                (new ParallelCommandGroup(getPathPlannerCommand())),
                                new InstantCommand(() -> turnOnBrakesDrivetrain(true)));
                if (driverStationTab.getAutoPath().autoBalance()) {
                        auto.addCommands(new AutoBalanceState(drivetrain));
                } 
                else if (driverStationTab.getAutoPath().Pickup()) {
                        auto.addCommands(new FaceScoreLocationState(drivetrain, 6),
                        arm.changeState(AutoPath::selectedPiece2nd, AutoPath::pickupLocation, true));
                                
                        auto.addCommands(new InstantCommand(() -> turnOnBrakesDrivetrain(false)),
                                new ParallelDeadlineGroup(getPickupPathPlannerCommand(),  intake.SpinIntakeCommand(()-> driverStationTab.getAutoPath().selectedPiece2nd(),true)));
                                        new InstantCommand(() -> turnOnBrakesDrivetrain(true));
                        
                        auto.addCommands(new Stow(arm, intake, false));
                        if (driverStationTab.getAutoPath().Return()) {
                                auto.addCommands(new FaceScoreLocationState(drivetrain, (180)));
                                auto.addCommands(new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                                new InstantCommand(() -> turnOnBrakesDrivetrain(false)),
                                                                getReturnPathPlannerCommand(),
                                                                new InstantCommand(() -> turnOnBrakesDrivetrain(true))),
                                                (new SequentialCommandGroup(new WaitCommand(2.5),
                                                                new SimpleScore(arm, intake,
                                                                                () -> PieceType.Cube,
                                                                               () ->  GenericPosition.Level2, true)))));
                        }
                }
                return auto;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                drivetrain.setDrivetrainSpeed(DrivetrainConstants.FAST_SPEED);
                drivetrain.setBrakeMode(IdleMode.kCoast);
                return SimpleAuto(driverStationTab.getAutoPath())
                                .andThen(() -> drivetrain.setBrakeMode(IdleMode.kBrake));
        }
}
