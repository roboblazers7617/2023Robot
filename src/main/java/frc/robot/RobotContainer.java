// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PickupLocation;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ScoreLevel;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.DrivetrainConstants.AutoPath;
import frc.robot.Constants.PnuematicsConstants.PnuematicPositions;
import frc.robot.Constants.WristConstants.WristPosition;
import frc.robot.commands.ArmStuff.IntakePiece;
import frc.robot.commands.ArmStuff.OutakePiece;
import frc.robot.commands.ArmStuff.SimpleMoveToPickup;
import frc.robot.commands.ArmStuff.SimpleMoveToScore;
import frc.robot.commands.ArmStuff.SimpleScore;
import frc.robot.commands.ArmStuff.Stow;
import frc.robot.commands.ArmStuff.ToggleArmPnuematics;
import frc.robot.commands.Drivetrain.AlignToDouble;
import frc.robot.commands.Drivetrain.AutoBalance;
import frc.robot.shuffleboard.ArmTab;
import frc.robot.shuffleboard.DriveTrainTab;
import frc.robot.shuffleboard.DriverStationTab;
import frc.robot.shuffleboard.IntakeTab;
import frc.robot.shuffleboard.ShuffleboardInfo;
import frc.robot.shuffleboard.ShuffleboardTabBase;
import frc.robot.shuffleboard.VisionTab;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Pnuematics;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        private final Wrist wrist = new Wrist();
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

        private PieceType selectedPiece = PieceType.CONE;

        private final DriverStationTab driverStationTab;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureDriverBindings();
                configureOperatorBindings();
                // create shuffleboardinfo.java
                drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(m_driverController.getLeftY(),
                                m_driverController.getRightX(), m_driverController.getRightY(),
                                () -> isRightTriggerPressed()),
                                drivetrain));

                arm.setDefaultCommand(new RunCommand(
                                () -> arm.setVelocity(
                                                (Math.abs(m_operatorController.getLeftY()) > OperatorConstants.DEADZONE
                                                                ? -m_operatorController.getLeftY()
                                                                : 0) * ArmConstants.MAX_MANNUAL_ARM_SPEED),
                                arm));
                wrist.setDefaultCommand(new RunCommand(() -> wrist.setVelocity(
                                (Math.abs(m_operatorController.getRightY()) > OperatorConstants.DEADZONE
                                                ? -m_operatorController.getRightY()
                                                : 0) * WristConstants.MAX_MANNUAL_WRIST_SPEED,
                                arm::getArmAngle), wrist));

                ArrayList<ShuffleboardTabBase> tabs = new ArrayList<>();
                // YOUR CODE HERE | | |
                // \/ \/ \/
                driverStationTab = new DriverStationTab(drivetrain, intake);
                tabs.add(driverStationTab);
                tabs.add(new VisionTab(vision, drivetrain));

                tabs.add(new DriveTrainTab(drivetrain));
                // tabs.add(new ColorSensorTab(new ColorSensor()));
                tabs.add(new IntakeTab(intake, wrist));
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

                // m_driverController.leftTrigger().whileTrue(new DriveToScoreGrid(drivetrain,
                // () -> m_driverController.getLeftY(),
                // () -> m_driverController.getRightY(),
                // () -> m_driverController.getRightX(),
                // DriverStation.getAlliance()));

                m_driverController.rightBumper()
                                .onTrue(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.SLOW_SPEED)));
                m_driverController.rightBumper()
                                .onFalse(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED)));

                m_driverController.leftBumper()
                                .onTrue(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.FAST_SPEED)));
                m_driverController.leftBumper()
                                .onFalse(new InstantCommand(
                                                () -> drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED)));

                // m_driverController.b().onTrue(new InstantCommand(() ->
                // drivetrain.resetOdometry(new Pose2d())));
                m_driverController.a().whileTrue(new AutoBalance(drivetrain));
                m_driverController.x().onTrue(new InstantCommand(() -> drivetrain.toggleBrakeMode()));
                Trigger doubleSubstationAlign = new Trigger(() -> (vision
                                .getBestTagDistance() > VisionConstants.IN_RANGE_OF_TAG)
                                && ((vision.getBestTagId() == VisionConstants.RED_PICKUP_STATION_TAG)
                                                || vision.getBestTagId() == VisionConstants.BLUE_PICKUP_STATION_TAG));
                doubleSubstationAlign.and(m_driverController.b().negate())
                                .onTrue(new AlignToDouble(vision, drivetrain, arm, wrist, intake,
                                                () -> getSelectedPiece(),
                                                m_driverController::getLeftY, m_driverController::getRightY,
                                                m_driverController::getRightX, () -> isRightTriggerPressed()))
                                .onFalse(new Stow(arm, wrist, intake));

        }

        private void configureOperatorBindings() {
                m_operatorController.leftBumper()
                                .onTrue(new SimpleMoveToPickup(arm, wrist, () -> getSelectedPiece(),
                                                () -> PickupLocation.DOUBLE));
                m_operatorController.leftTrigger()
                                .onTrue(new SimpleMoveToPickup(arm, wrist, () -> getSelectedPiece(),
                                                () -> PickupLocation.FLOOR));

                m_operatorController.rightBumper()
                                .onTrue(Commands.runOnce(() -> setSelectedPiece(PieceType.CONE)));
                m_operatorController.rightTrigger()
                                .onTrue(Commands.runOnce(() -> setSelectedPiece(PieceType.CUBE)));

                m_operatorController.povLeft()
                                .onTrue(new Stow(arm, wrist, intake));

                m_operatorController.y().whileTrue(new IntakePiece(intake, () -> getSelectedPiece()));

                m_operatorController.x()
                                .whileTrue(new OutakePiece(intake, () -> getSelectedPiece()));

                m_operatorController.a().onTrue(new ToggleArmPnuematics(arm));

                m_operatorController.povDown().onTrue(
                                new SimpleMoveToScore(arm, wrist, () -> ScoreLevel.LEVEL_1, () -> getSelectedPiece()));
                m_operatorController.povRight().onTrue(
                                new SimpleMoveToScore(arm, wrist, () -> ScoreLevel.LEVEL_2, () -> getSelectedPiece()));
                m_operatorController.povUp().onTrue(
                                new SimpleMoveToScore(arm, wrist, () -> ScoreLevel.LEVEL_3, () -> getSelectedPiece()));
        }

        // private void setScoreLevel(ScoreLevel scoreLevel) {
        // this.scoreLevel = scoreLevel;
        // }

        public void stow() {
                arm.setPosition(ArmPositions.STOW);
                wrist.setPosition(WristPosition.STOW, () -> arm.getArmAngle());
                arm.actuateSuperstructure(PnuematicPositions.RETRACTED);
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
                if (piece == PieceType.CONE) {
                        leds.orange();
                } else {
                        leds.purple();
                }
                selectedPiece = piece;
        }

        public void turnOnMechanismBrakes(Boolean isBraked) {
                wrist.turnOnBrakes(isBraked);
                arm.turnOnBrakes(isBraked);
        }

        public void turnOnBrakesDrivetrain(Boolean isTrue) {
                drivetrain.turnOnBrakes(isTrue);
        }

        private boolean isRightTriggerPressed() {
                return m_driverController.getRightTriggerAxis() > 0.5;
        };

        public Command getPathPlannerCommand() {
                PathPlannerTrajectory path = PathPlanner.loadPath(driverStationTab.getAutoPath().pathname(),
                                new PathConstraints(2,
                                                1.5),
                                driverStationTab.getAutoPath().isReverse());

                drivetrain.resetOdometry(path.getInitialPose());
                return new PPRamseteCommand(path,
                                drivetrain::getPose2d,
                                new RamseteController(DrivetrainConstants.RAMSETEb, DrivetrainConstants.RAMSETEzeta),
                                new SimpleMotorFeedforward(DrivetrainConstants.KS, DrivetrainConstants.KV,
                                                DrivetrainConstants.KA),
                                drivetrain.getKinematics(),
                                drivetrain::getWheelSpeeds,
                                new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN,
                                                DrivetrainConstants.KD_LIN),
                                new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN,
                                                DrivetrainConstants.KD_LIN),
                                drivetrain::tankDriveVolts,
                                false,
                                drivetrain);
        }

        public SequentialCommandGroup SimpleAuto(AutoPath AutoPath) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                if (driverStationTab.getAutoPath().scoring()) {
                        if (driverStationTab.getAutoPath().autoBalance()) {
                                return new SequentialCommandGroup(
                                                new SimpleScore(arm, wrist, intake,
                                                                () -> driverStationTab.getAutoPath().selectedPiece(),
                                                                () -> driverStationTab.getAutoPath().scoreLevelFirst()),
                                                new Stow(arm, wrist, intake),
                                                new InstantCommand(() -> turnOnBrakesDrivetrain(false)),
                                                getPathPlannerCommand(),
                                                new InstantCommand(() -> turnOnBrakesDrivetrain(true)),
                                                new AutoBalance(drivetrain));
                        } else {
                                return new SequentialCommandGroup(
                                                new SimpleScore(arm, wrist, intake,
                                                                () -> driverStationTab.getAutoPath().selectedPiece(),
                                                                () -> driverStationTab.getAutoPath().scoreLevelFirst()),
                                                new Stow(arm, wrist, intake),
                                                new InstantCommand(() -> turnOnBrakesDrivetrain(false)),
                                                getPathPlannerCommand(),
                                                new InstantCommand(() -> turnOnBrakesDrivetrain(true)));
                        }
                } else if (driverStationTab.getAutoPath().autoBalance()) {
                        return new SequentialCommandGroup(
                                        new InstantCommand(() -> turnOnBrakesDrivetrain(false)),
                                        getPathPlannerCommand(),
                                        new InstantCommand(() -> turnOnBrakesDrivetrain(true)),
                                        new AutoBalance(drivetrain));
                } else {
                        return new SequentialCommandGroup(
                                        new InstantCommand(() -> turnOnBrakesDrivetrain(false)),
                                        getPathPlannerCommand(),
                                        new InstantCommand(() -> turnOnBrakesDrivetrain(true)));
                }
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                drivetrain.setBrakeMode(IdleMode.kCoast);
                return SimpleAuto(driverStationTab.getAutoPath())
                                .andThen(() -> drivetrain.setBrakeMode(IdleMode.kBrake));
        }
        // TODO Sam, I need to see If I can find a way to delete the extra command named
        // null in "red far 2 ball"

        // private List<PathPlannerTrajectory> pathGroup =
        // PathPlanner.loadPathGroup("red near 2 ball", new
        // PathConstraints(DrivetrainConstants.MAX_AUTO_VELOCITY,
        // DrivetrainConstants.MAX_AUTO_ACCELERATION));

        // public Command pickAutonomousCommand(DrivetrainConstants.AutoPath autopath) {
        // // the hashmap can really just be in constants, it does not need to be here
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("Stow", new Stow(arm, wrist, intake));
        // eventMap.put("AutoBalance", new AutoBalance(drivetrain));
        // eventMap.put("SimplePickup", new SimplePickup(arm, wrist, intake, () ->
        // autopath.selectedPiece(),
        // () -> autopath.pickupLocation()));
        // eventMap.put("SimpleScore", new SimpleScore(arm, wrist, intake, () ->
        // autopath.selectedPiece(),
        // () -> autopath.scoreLevelSecond()));

        // //The code below is used to define a singular path for the robot to follow
        // PathPlannerTrajectory test_path = PathPlanner.loadPath(
        // autopath.pathname(), new
        // PathConstraints(DrivetrainConstants.MAX_AUTO_VELOCITY,
        // DrivetrainConstants.MAX_AUTO_ACCELERATION),
        // autopath.isReverse());
        // drivetrain.resetOdometry(test_path.getInitialPose());
        // //this is the autoBuilder, in theory we do not need this to be here, and we
        // should put it somewher else
        // RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
        // drivetrain::getPose2d, // Pose2d supplier
        // drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the
        // beginning of auto
        // new RamseteController(DrivetrainConstants.RAMSETEb,
        // DrivetrainConstants.RAMSETEzeta),
        // drivetrain.getKinematics(),
        // new SimpleMotorFeedforward(DrivetrainConstants.KS_LIN,
        // DrivetrainConstants.KV),
        // () -> drivetrain.getWheelSpeeds(), // WheelSpeeds supplier
        // new PIDConstants(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN,
        // DrivetrainConstants.KD_LIN),
        // // PID constants to correct for rotation error (used to create the rotation
        // controller)
        // drivetrain::tankDriveVolts, // Module states consumer used to output to the
        // drive subsystem
        // eventMap,
        // false, // Should the path be automatically mirrored depending on alliance
        // color, Optional, defaults to true
        // drivetrain // The drive subsystem. Used to properly set the requirements of
        // path following commands
        // );

        // return autoBuilder.fullAuto(pathGroup);
        // }
}
