// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class FaceTarget extends CommandBase {
  /** Creates a new FaceTarget. */
  private Drivetrain drivetrain;
  private Pose2d targetPose;
  private double angleToGoal;
  private Supplier<Pose2d> targetPoseSupplier;
  private PIDController pidController;
  public FaceTarget(Drivetrain drivetrain, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    addRequirements(drivetrain);
    pidController = new PIDController(DrivetrainConstants.KP_ROT, DrivetrainConstants.KI_ROT, DrivetrainConstants.KD_ROT);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    pidController.setTolerance(Units.degreesToRadians(0.666666666666666666666666666666666666));
    
  }
  
  public FaceTarget(Drivetrain drivetrain, Supplier<Pose2d> targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPose;
    addRequirements(drivetrain);
    pidController = new PIDController(DrivetrainConstants.KP_ROT, DrivetrainConstants.KI_ROT, DrivetrainConstants.KD_ROT);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    pidController.setTolerance(Units.degreesToRadians(3.0));
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    if(targetPoseSupplier != null){
      targetPose = targetPoseSupplier.get();
    }
    pidController.reset();
    if(targetPoseSupplier != null){
      targetPose = targetPoseSupplier.get();
    }
    angleToGoal = targetPose.getTranslation().minus(drivetrain.getPose2d().getTranslation()).getAngle().getRadians();
  pidController.setSetpoint(angleToGoal);
  //System.out.println("Starting Position" + drivetrain.getPose2d().getX() +  "Y" + drivetrain.getPose2d().getY());
  //System.out.println("target Position" + targetPose.getX() +  "Y" + targetPose.getY());
  //System.out.println("HERE, HERE, HERE" + angleToGoal);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pidController.calculate(drivetrain.getRotation2d().getRadians());
    System.out.println("Angular speed is " + output);
    System.out.println("Setpoint is " + Units.radiansToDegrees(pidController.getSetpoint()));
    System.out.println("Drivetrain value is " + drivetrain.getPose2d().getRotation().getRadians());
    //drivetrain.arcadeDrive(0, MathUtil.clamp(output, -DrivetrainConstants.MAX_ANGULAR_VELOCITY, DrivetrainConstants.MAX_ANGULAR_VELOCITY));
    drivetrain.driveWithVelocity(0, /*Math.copySign(.15, output)+*/output );
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

  
}
