// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PickupLocation;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ArmStuff.SimpleMoveToPickup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToDouble extends CommandBase {
  Drivetrain mDrivetrain;
  Vision mVision;
  DoubleSupplier mLeftY;
  DoubleSupplier mRightY;
  DoubleSupplier mRightX;
  Supplier<Boolean> mIsQuickTurn;

  /** Creates a new AlignToDouble. */
  public AlignToDouble(Vision vision, Drivetrain drivetrain, DoubleSupplier leftY, DoubleSupplier rightY, DoubleSupplier rightX, Supplier<Boolean> isQuickTurn) {
    addRequirements(drivetrain);
    mDrivetrain = drivetrain;
    mVision = vision;
    mLeftY = leftY;
    mRightY = rightY;
    mRightX = rightX;
    mIsQuickTurn = isQuickTurn;

  }

  @Override
  public void initialize() {
    mDrivetrain.setDrivetrainSpeed(DrivetrainConstants.SLOW_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!mVision.inRangeOfDoubleStationStop() && mVision.inRangeOfDoubleStation()){
      mDrivetrain.drive(mLeftY, mRightX, mRightY, mIsQuickTurn);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.setDrivetrainSpeed(DrivetrainConstants.REG_SPEED);
    mDrivetrain.driveWithVelocity(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
