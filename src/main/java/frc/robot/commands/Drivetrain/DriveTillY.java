// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DrivetrainConstants;
public class DriveTillY extends CommandBase {
  /** Creates a new SmartDriver. */
  private Drivetrain drivetrain;
  private DoubleSupplier leftY;
  private DoubleSupplier rightY;
  private DoubleSupplier rightX;
  private Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;
  public DriveTillY(Drivetrain drivetrain, DoubleSupplier leftY, DoubleSupplier rightY, DoubleSupplier rightX, Supplier<Pose2d> targetPoseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.leftY = leftY;
    this.rightY = rightY;
    this.rightX = rightX;
    this.targetPoseSupplier = targetPoseSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = targetPoseSupplier.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((targetPose.getY() - Units.inchesToMeters(24.0)) <= drivetrain.getPose2d().getY() && (targetPose.getY() + Units.inchesToMeters(24.0)) >= drivetrain.getPose2d().getY()) 
    {
    drivetrain.setDrivetrainSpeed(DrivetrainConstants.SLOW_SPEED);
    }  
    drivetrain.drive(leftY.getAsDouble(), rightX.getAsDouble(), rightY.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((targetPose.getY() - DrivetrainConstants.MAX_ERROR_LINEAR) <= drivetrain.getPose2d().getY() && (targetPose.getY() + DrivetrainConstants.MAX_ERROR_LINEAR) >= drivetrain.getPose2d().getY()) 
      return true;
    return false;
  }
}
