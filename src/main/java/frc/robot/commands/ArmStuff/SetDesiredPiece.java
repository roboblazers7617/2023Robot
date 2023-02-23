// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.armIntakeCordinatorUtil;
import frc.robot.armIntakeCordinatorUtil.PieceType;

public class SetDesiredPiece extends CommandBase {
  /** Creates a new SetDesiredPiece. */
  PieceType mPiece;
  armIntakeCordinatorUtil mCordinatorUtil;
  public SetDesiredPiece(PieceType piece, armIntakeCordinatorUtil cordinatorUtil) {
    // Use addRequirements() here to declare subsystem dependencies.
    mPiece = piece;
    mCordinatorUtil = cordinatorUtil;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mCordinatorUtil.setDesiredOrHeldPiece(mPiece);
    //TODO: Use LED Methods to change to color of desired piece
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
