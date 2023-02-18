// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class GoToTarget extends CommandBase {
  /** Creates a new GoToTarget. */
  private Drivetrain drivetrain;
  private Supplier<Pose2d> targetPoseSupplier;
  private Pose2d targetPose;
  private PIDController pidController;
  private PIDController turnPidController;
  private Translation2d startTranslation2d;
  private double distanceToGoal;
  private double endingEncoderValue;
  private Alliance color;

  private double linOutput;
  private double rotOutput;

  private double linSimpleFF;
  private double rotSimpleFF;
  
  public GoToTarget(Drivetrain drivetrain, Pose2d targetPose, Alliance color) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    this.color = color;

    pidController = new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN);
    pidController.setTolerance(DrivetrainConstants.ERROR_TARGET_DRIVER);

    turnPidController = new PIDController(DrivetrainConstants.KP_ROT, DrivetrainConstants.KI_ROT, DrivetrainConstants.KD_ROT);
    turnPidController.setTolerance(1);
  }
  public GoToTarget(Drivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier, Alliance color) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPoseSupplier;
    this.color = color;

    pidController = new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN);
    pidController.setTolerance(DrivetrainConstants.ERROR_TARGET_DRIVER);

    turnPidController = new PIDController(DrivetrainConstants.KP_ROT, DrivetrainConstants.KI_ROT, DrivetrainConstants.KD_ROT);
    turnPidController.enableContinuousInput(-180, 180);
    turnPidController.setTolerance(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double targetAngle;
    if (targetPoseSupplier != null) {
      targetPose = targetPoseSupplier.get();
    }
    startTranslation2d = drivetrain.getPose2d().getTranslation();
    distanceToGoal = targetPose.getTranslation().getDistance(startTranslation2d);

    endingEncoderValue = drivetrain.getaverageEncoderDistance() + distanceToGoal;
    pidController.setSetpoint(endingEncoderValue);

    if (color == Alliance.Blue)
    {
      targetAngle = 180.0;
    }
    else
    {
      targetAngle = 0.0;
    }
   turnPidController.setSetpoint(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     linOutput = pidController.calculate(drivetrain.getaverageEncoderDistance());
     rotOutput = turnPidController.calculate(drivetrain.getRotation2d().getDegrees());

     linSimpleFF = Math.copySign(0.5, linOutput);//0.3
     rotSimpleFF = Math.copySign(0.3, rotOutput);

    drivetrain.arcadeDrive( 
    MathUtil.clamp(linSimpleFF + linOutput, -DrivetrainConstants.MAX_LINEAR_VELOCITY , DrivetrainConstants.MAX_LINEAR_VELOCITY), 
    MathUtil.clamp((turnPidController.atSetpoint()) ? 0 : (rotOutput + rotSimpleFF), -DrivetrainConstants.MAX_ANGULAR_VELOCITY , DrivetrainConstants.MAX_ANGULAR_VELOCITY));
    //drivetrain.driveWithVelocity(linSimpleFF + linOutput, (turnPidController.atSetpoint()) ? 0 : (rotOutput + rotSimpleFF));
    System.out.println("Velocity is " + linOutput + " FF " + linSimpleFF);
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
