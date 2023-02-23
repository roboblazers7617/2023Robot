// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.armIntakeCordinatorUtil;
import frc.robot.Constants.IntakeConstants.IntakeDirection;
import frc.robot.armIntakeCordinatorUtil.PieceType;
import frc.robot.subsystems.Intake;

public class IntakePiece extends CommandBase {
  /** Creates a new IntakePiece. */
  Intake mIntake;
  armIntakeCordinatorUtil mCordinatorUtil;
  public IntakePiece(Intake intake, armIntakeCordinatorUtil cordinatorUtil) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mCordinatorUtil = cordinatorUtil;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mCordinatorUtil.getDesiredOrHeldPiece() == PieceType.CONE)
      mIntake.setIntakeSpeed(IntakeDirection.PickCone.speed());
    if(mCordinatorUtil.getDesiredOrHeldPiece() == PieceType.CUBE)
      mIntake.setIntakeSpeed(IntakeDirection.PickCube.speed());
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.setIntakeSpeed(IntakeDirection.Stop.speed());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
