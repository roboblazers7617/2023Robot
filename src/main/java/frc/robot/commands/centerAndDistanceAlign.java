// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class centerAndDistanceAlign extends CommandBase {
  /** Creates a new centerAndDistanceAlign. */
  Drivetrain mDrivetrain;
  Vision mVision;
  double distanceToAlign;
  boolean done = false;
  PIDController rotationalController = new PIDController(DrivetrainConstants.KP_ROT, DrivetrainConstants.KI_ROT, DrivetrainConstants.KD_ROT);
  PIDController distanceController = new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN);
  public centerAndDistanceAlign(Vision vision, Drivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    mDrivetrain = drivetrain;
    mVision = vision;
    distanceToAlign = distance;
    done = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationalController.enableContinuousInput(-180, 180);
    rotationalController.setTolerance(3);
    distanceController.setTolerance(0.03);
    rotationalController.setSetpoint(0);
    distanceController.setSetpoint(2);
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(mVision.getBestTagDistance() > 0){   
    mDrivetrain.arcadeDrive(MathUtil.clamp(-distanceController.calculate(mVision.getBestTagDistance(),distanceToAlign), -.5, .5),
     MathUtil.clamp(-rotationalController.calculate(mVision.getBestTagYaw()), -.5, .5));
     }
    else{
      mDrivetrain.arcadeDrive(0, 0);
    }



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
   // return (rotationalController.atSetpoint() && distanceController.atSetpoint());
  }
}
