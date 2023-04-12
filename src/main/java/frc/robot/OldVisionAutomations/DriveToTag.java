// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OldVisionAutomations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class DriveToTag extends CommandBase {
  PIDController controller;
  Vision mVision;
  Drivetrain mDrivetrain;

  /** Creates a new DriveToTag. */
  public DriveToTag(Vision vision, Drivetrain drivetrain, double distanceMeters) {
    // The controller that the command will use
    controller = new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN);
    controller.setSetpoint(distanceMeters);

    //TODO: Lukas. (Medium) Please put number in constant
    controller.setTolerance(.03);
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivetrain = drivetrain;
    mVision = vision;
    //TODO: Lukas. (High) Does this need to require vision?
    addRequirements(drivetrain);

    //TODO: Lukas. (High) Please put number in constant
    drivetrain.setDrivetrainSpeed(1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mVision.getBestTagDistance() > 0)
    //TODO: Lukas. (High) You use KS_LIN here, but perhaps in a different way than in other subsystems. 
    // There is a constant SIMPLE_FF_LINEAR that is used in DriveForwardToScoreLocation that might be more approrpriate. But that is set to a different value.
      mDrivetrain.arcadeDrive(-MathUtil.clamp(controller.calculate(mVision.getBestTagDistance()) + Math.copySign(DrivetrainConstants.KS_LIN, controller.calculate(mVision.getBestTagDistance())), -0.5, .5), 0);
    else
      mDrivetrain.arcadeDrive(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
