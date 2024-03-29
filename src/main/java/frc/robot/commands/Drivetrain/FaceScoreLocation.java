// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.lang.constant.DirectMethodHandleDesc;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class FaceScoreLocation extends CommandBase {
  /** Creates a new FaceScoreLocation. */
  private PIDController turnController = 
        new PIDController(DrivetrainConstants.KP_ROT_POS, DrivetrainConstants.KI_ROT_POS, DrivetrainConstants.KD_ROT_POS);
  private Drivetrain drivetrain;
  private Alliance color;
  private double targetAngle;
  private final double angleTolerance = DrivetrainConstants.MAX_ERROR_ROTATION;
  private boolean manuelAngle = false;

  public FaceScoreLocation(Drivetrain drivetrain, Alliance color) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.color = color;
    turnController.setTolerance(angleTolerance);
    turnController.enableContinuousInput(-180, 180);
    addRequirements(drivetrain);

  }
  public FaceScoreLocation(Drivetrain drivetrain, double targetAngle) {
    manuelAngle = true;
    this.drivetrain = drivetrain;
    this.targetAngle = targetAngle;
    turnController.setTolerance(angleTolerance);
    turnController.enableContinuousInput(-180, 180);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED);
    if (manuelAngle == false){
      if (color == Alliance.Blue)
      {
        targetAngle = DrivetrainConstants.ALLIANCE_BLUE_ROTATION;
      }
      else
      {
        targetAngle = DrivetrainConstants.ALLIANCE_RED_ROTATION;
      }
    turnController.setSetpoint(targetAngle);
    }
    else {
      turnController.setSetpoint(targetAngle);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Gyro is " + drivetrain.getRotation2d().getDegrees());

    double output = turnController.calculate(drivetrain.getRotation2d().getDegrees());
    double simpleFF = Math.copySign(0.4, output);
    // System.out.println("Speed is " + output);
    drivetrain.arcadeDrive(0.0,MathUtil.clamp(output+ simpleFF, -DrivetrainConstants.MAX_ANGULAR_VELOCITY, DrivetrainConstants.MAX_ANGULAR_VELOCITY));
    //System.out.println("**** Executing Turn: output is " + output + "    target angle is " + targetAngle + "  current angle is " + drivetrain.getRotation2d().getDegrees());
    //System.out.println("Simple FF is " + simpleFF); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0.0, 0.0);
    drivetrain.setDrivetrainSpeed(DrivetrainConstants.FAST_SPEED);
    System.out.println ("I DID SOMETHING Gyro angle is " + drivetrain.getRotation2d().getDegrees());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // boolean isDone = false;
    // double angle = Math.abs(drivetrain.getRotation2d().getDegrees());
    // if ((angle >= (targetAngle - angleTolerance)) 
    //           &&  (angle <= (targetAngle + angleTolerance)))
    // {
    //   isDone = true;
    // }

    // return isDone;
    return turnController.atSetpoint();
  }
}
