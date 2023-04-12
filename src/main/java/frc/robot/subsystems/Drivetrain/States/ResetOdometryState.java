// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain.States;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.team4272.globals.State;

public class ResetOdometryState extends State<Drivetrain> {
  /** Creates a new FollowPathState. */
  Pose2d pose;
  public ResetOdometryState(Drivetrain drivetrain, PathPlannerTrajectory path) {
    super(drivetrain);
    pose = path.getInitialPose();
  }

  public ResetOdometryState(Drivetrain drivetrain, Pose2d pose) {
    super(drivetrain);
    this.pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    requiredSubsystem.resetOdometry(pose);
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
