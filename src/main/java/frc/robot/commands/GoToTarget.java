// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class GoToTarget extends CommandBase {
  /** Creates a new GoToTarget. */
  private Drivetrain drivetrain;
  private Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;
  private PIDController pidController;
  private Translation2d startTranslation2d;
  private double distanceToGoal;
  private double endingEncoderValue;
  
  public GoToTarget(Drivetrain drivetrain, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    pidController = new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN);
    pidController.setTolerance(DrivetrainConstants.ERROR_TARGET_DRIVER);
  }
  public GoToTarget(Drivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPoseSupplier;
    pidController = new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN);
    pidController.setTolerance(DrivetrainConstants.ERROR_TARGET_DRIVER);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (targetPoseSupplier != null) {
      targetPose = targetPoseSupplier.get();
    }
    startTranslation2d = drivetrain.getPose2d().getTranslation();
    distanceToGoal = targetPose.getTranslation().getDistance(startTranslation2d);
    endingEncoderValue = drivetrain.getaverageEncoderDistance() + distanceToGoal;
    pidController.setSetpoint(endingEncoderValue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(drivetrain.getaverageEncoderDistance());
    //drivetrain.arcadeDrive(0.2 + MathUtil.clamp(output, -DrivetrainConstants.MAX_LINEAR_VELOCITY , DrivetrainConstants.MAX_LINEAR_VELOCITY), 0);
    drivetrain.driveWithVelocity(output, 0);
    //System.out.println(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("start" + startTranslation2d);
    //System.out.println("Dist" + distanceToGoal);
    return pidController.atSetpoint();
  }
}
