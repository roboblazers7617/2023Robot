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
  double rotOutput;
  double linOutput;
  PIDController rotationalController = new PIDController(DrivetrainConstants.KP_ROT_POS, DrivetrainConstants.KI_ROT_POS, DrivetrainConstants.KD_ROT_POS);
  PIDController distanceController = new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN);
  public centerAndDistanceAlign(Vision vision, Drivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    //TODO: Lukas. (High) Does this need to require Vision?
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
    //TODO: Lukas. Move magic numbers into constants
    rotationalController.setTolerance(3);
    distanceController.setTolerance(0.03);
    rotationalController.setSetpoint(0);
    distanceController.setSetpoint(distanceToAlign);
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotOutput = rotationalController.calculate(mVision.getBestTagYaw());
    linOutput = distanceController.calculate(mVision.getBestTagDistance());
    if(mVision.getBestTagDistance() > 0){   
    //TODO: Lukas. (High) You use KS_LIN here, but perhaps in a different way than in other subsystems. 
    // There is a constant SIMPLE_FF_LINEAR that is used in DriveForwardToScoreLocation that might be more approrpriate. But that is set to a different value.
    mDrivetrain.arcadeDrive(-MathUtil.clamp((linOutput + Math.copySign(DrivetrainConstants.KS_LIN, linOutput)), -0.5, .5),
    //TODO: Lukas. (High) same as above but with KS_ROT. Use SIMPLE_FF_ANGULAR
    MathUtil.clamp(rotOutput+Math.copySign(DrivetrainConstants.KS_ROT, rotOutput), -.5, .5));
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

    return (rotationalController.atSetpoint() && distanceController.atSetpoint());
  }
}
