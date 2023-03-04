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
  PIDController controller = new PIDController(DrivetrainConstants.KP_BALANCE, DrivetrainConstants.KI_BALANCE,
      DrivetrainConstants.KD_BALANCE);

  public AutoBalance(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivetrain = drivetrain;
    addRequirements(drivetrain);
    //TODO: Lukas. (High) Make this a constant
    controller.setSetpoint(3);
    controller.setTolerance(DrivetrainConstants.BALANCING_TOLERANCE);
    controller.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivetrain.arcadeDrive(
        MathUtil.clamp(controller.calculate(mDrivetrain.getPitch()), -DrivetrainConstants.MAX_BALANCE_SPEED,
            DrivetrainConstants.MAX_BALANCE_SPEED),
        0);
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