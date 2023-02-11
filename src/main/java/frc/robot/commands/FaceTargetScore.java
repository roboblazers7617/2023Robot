// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class FaceTargetScore extends CommandBase {
  /** Creates a new FaceTarget. */
  private Drivetrain drivetrain;
  private Pose2d targetPose;
  private double angleToGoal;
  private PIDController pidController;
  public FaceTargetScore(Drivetrain drivetrain, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    addRequirements(drivetrain);
    pidController = new PIDController(DrivetrainConstants.KP_ROT, DrivetrainConstants.KI_ROT, DrivetrainConstants.KD_ROT);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(DrivetrainConstants.ROTATIONAL_ERROR_TARGET_DRIVER);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    angleToGoal = drivetrain.findTargetAngle(targetPose.getX(), targetPose.getY(), drivetrain.getPose2d().getX(), drivetrain.getPose2d().getX());
  pidController.setSetpoint(180);
  System.out.println("Starting Position" + drivetrain.getPose2d().getX() +  "Y" + drivetrain.getPose2d().getY());
  System.out.println("target Position" + targetPose.getX() +  "Y" + targetPose.getY());
  System.out.println("HERE, HERE, HERE" + angleToGoal);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(drivetrain.getRotation2d().getDegrees());
    drivetrain.arcadeDrive(0, MathUtil.clamp(output, -DrivetrainConstants.MAX_ANGULAR_VELOCITY, DrivetrainConstants.MAX_ANGULAR_VELOCITY));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }

  public double findTheta(double xTarget, double yTarget, double xStart, double yStart){
    double angle = (Math.toDegrees(Math.atan2(xTarget - xStart, -(yTarget - yStart))) - 90);
    //angle = angle - drivetrain.getRotation2d().getDegrees();
    SmartDashboard.putNumber("ANGLE", angle <= 180? angle: (angle -360));
    return angle <= 180? angle: (angle -360);

  }
}
