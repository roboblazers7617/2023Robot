// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DrivetrainConstants;
public class SmartDriver extends CommandBase {
  /** Creates a new SmartDriver. */
  private Drivetrain drivetrain;
  private DoubleSupplier leftY;
  private DoubleSupplier rightY;
  private DoubleSupplier rightX;
  private Pose2d targetPose;
  public SmartDriver(Drivetrain drivetrain, DoubleSupplier leftY, DoubleSupplier rightY, DoubleSupplier rightX, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.leftY = leftY;
    this.rightY = rightY;
    this.rightX = rightX;
    this.targetPose = targetPose;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(leftY.getAsDouble(), rightX.getAsDouble(), rightY.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((targetPose.getY() - DrivetrainConstants.ERROR_TARGET_DRIVER) <= drivetrain.getPose2d().getY() && (targetPose.getY() + DrivetrainConstants.ERROR_TARGET_DRIVER) >= drivetrain.getPose2d().getY()) 
      return true;
    return false;
  }
}
