// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class GoToTarget extends CommandBase {
  /** Creates a new GoToTarget. */
  private Drivetrain drivetrain;
  private Pose2d targetpose;
  private PIDController pidController;
  private Translation2d startTranslation2d;
  private double distanceToGoal;
  public GoToTarget(Drivetrain drivetrain, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.targetpose = targetpose;
    pidController = new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN);
    pidController.setTolerance(DrivetrainConstants.LINEAR_ERROR_TARGET_DRIVER);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTranslation2d = drivetrain.getPose2d().getTranslation();
    distanceToGoal = targetpose.getTranslation().getDistance(startTranslation2d);
    pidController.setSetpoint(targetpose.getX());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(drivetrain.getPose2d().getTranslation().getX());
    drivetrain.arcadeDrive(output, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
