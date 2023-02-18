// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class FaceToTargetScore extends CommandBase {
  /** Creates a new FaceTarget. */
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DrivetrainConstants.KS,
      DrivetrainConstants.KV, DrivetrainConstants.KA);
  private Drivetrain drivetrain;
  private double angleToGoal;
  private Alliance color;
  private PIDController pidController;
  public FaceToTargetScore(Drivetrain drivetrain, Alliance color) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.color = color;
    addRequirements(drivetrain);
    pidController = new PIDController(DrivetrainConstants.KP_ROT, DrivetrainConstants.KI_ROT, DrivetrainConstants.KD_ROT);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    pidController.setTolerance(0.0087);
    
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    if (color == Alliance.Blue){
      angleToGoal = Units.degreesToRadians(180);
    }
    else{
      angleToGoal = Units.degreesToRadians(0);
    }
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
    //drivetrain.driveWithVelocity(0, /*Math.copySign(.15, output)+*/output );
    var speed = feedForward.calculate(output);
    drivetrain.arcadeDrive(0, Math.copySign(0.1, output)+ MathUtil.clamp(speed, -DrivetrainConstants.MAX_ANGULAR_VELOCITY, DrivetrainConstants.MAX_ANGULAR_VELOCITY));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isDone = false;
    double heading = Math.abs(drivetrain.getRotation2d().getDegrees());

    if ((heading >= 179.0 ) && (heading <= 181.0))
    {
      isDone = true;
    }
    
    return isDone;
  }

  
}
