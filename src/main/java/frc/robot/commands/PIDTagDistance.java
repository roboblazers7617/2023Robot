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
//TODO:This Function is Broken
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDTagDistance extends CommandBase {
  PIDController controller;
  Vision mVision;
  Drivetrain mDrivetrain;
  /** Creates a new distanceFromTag. */
  public PIDTagDistance(Vision vision, Drivetrain drivetrain, double distanceMeters) {
        // The controller that the command will use
        controller = new PIDController(DrivetrainConstants.KP_LIN, DrivetrainConstants.KI_LIN, DrivetrainConstants.KD_LIN);
        controller.setSetpoint(distanceMeters);
        controller.setTolerance(.03);
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivetrain = drivetrain;
    mVision = vision;
    addRequirements(drivetrain);
  }

  @Override
  public void execute(){
    /*if(mVision.getBestTagDistance() > 0)
      mDrivetrain.arcadeDrive(MathUtil.clamp(controller.calculate(mVision.getBestTagDistance())+.2, -0.5, .5), 0);
    else 
      mDrivetrain.arcadeDrive(0, 0);
      */
      mDrivetrain.arcadeDrive(.2, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//controller.atSetpoint();
  }
}
