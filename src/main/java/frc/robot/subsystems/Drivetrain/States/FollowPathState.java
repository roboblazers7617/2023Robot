// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain.States;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.team4272.globals.State;

public class FollowPathState extends State<Drivetrain> {
  /** Creates a new FollowPathState. */
  PathPlannerTrajectory path;
  public FollowPathState(Drivetrain drivetrain, PathPlannerTrajectory path) {
    super(drivetrain);
    this.path = path;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.andThen(new PPRamseteCommand(path,
    requiredSubsystem::getPose2d,
    new RamseteController(DrivetrainConstants.RAMSETEb, DrivetrainConstants.RAMSETEzeta),
    new SimpleMotorFeedforward(DrivetrainConstants.KS, DrivetrainConstants.KV,
                    DrivetrainConstants.KA),
    requiredSubsystem.getKinematics(),
    requiredSubsystem::getWheelSpeeds,
    new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN,
                    DrivetrainConstants.KD_LIN),
    new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN,
                    DrivetrainConstants.KD_LIN),
    requiredSubsystem::tankDriveVolts,
    false,
    requiredSubsystem));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
