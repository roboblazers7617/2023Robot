// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  Drivetrain mDrivetrain;
  PIDController controller = new PIDController(3, DrivetrainConstants.KI_BALANCE,
      DrivetrainConstants.KD_BALANCE);

  public AutoBalance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivetrain = drivetrain;
    addRequirements(drivetrain);
    controller.setSetpoint(0);
    controller.setTolerance(DrivetrainConstants.BALANCING_TOLERANCE);
    controller.enableContinuousInput(-180, 180);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.turnOnBrakes(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mDrivetrain.getPitch() < -DrivetrainConstants.BALANCE_SPEED_BOOST_TOLERANCE) {
      mDrivetrain.arcadeDrive(
          MathUtil.clamp(controller.calculate(mDrivetrain.getPitch() + .2), -DrivetrainConstants.MAX_BALANCE_SPEED,
              DrivetrainConstants.MAX_BALANCE_SPEED),
          0);
    } else if (mDrivetrain.getPitch() > DrivetrainConstants.BALANCE_SPEED_BOOST_TOLERANCE) {
      mDrivetrain.arcadeDrive(
          MathUtil.clamp(controller.calculate(mDrivetrain.getPitch() - DrivetrainConstants.BALANCE_OFFEST), -DrivetrainConstants.MAX_BALANCE_SPEED,
              DrivetrainConstants.MAX_BALANCE_SPEED),
          0);
    } else {
      mDrivetrain.arcadeDrive(
        MathUtil.clamp(controller.calculate(mDrivetrain.getPitch()), -DrivetrainConstants.MAX_BALANCE_SPEED,
            DrivetrainConstants.MAX_BALANCE_SPEED),
        0);
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
    return controller.atSetpoint();
  }
}
