// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class CenterRelativeTag extends CommandBase {
  Vision mVision;
  Drivetrain mDrivetrain;
  double distance;
  Pose2d pointToAlignOn;
  PIDController controller;
  double output;

  /** Creates a new CenterRetativeTag. */
  public CenterRelativeTag(Vision vision, Drivetrain drivetrain, double distanceMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    mVision = vision;
    mDrivetrain = drivetrain;
    addRequirements(drivetrain);
    controller.setTolerance(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = new Pose3d(0, distance, 0, new Rotation3d()).plus(mVision.getTransformToTag()).toPose2d();
    double angleToTurnTo = Units.radiansToDegrees(mDrivetrain.getPose2d().log(pose).dtheta);
    controller = new PIDController(DrivetrainConstants.KP_ROT, DrivetrainConstants.KI_ROT, DrivetrainConstants.KD_ROT);
    controller.setSetpoint(angleToTurnTo);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    output = controller.calculate(mDrivetrain.getPose2d().getRotation().getDegrees());
    mDrivetrain.arcadeDrive(0.0,MathUtil.clamp(output+Math.copySign(DrivetrainConstants.KS_ROT, output), -.5, .5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
