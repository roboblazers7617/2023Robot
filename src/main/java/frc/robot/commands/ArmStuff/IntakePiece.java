// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Intake;


public class IntakePiece extends CommandBase {
  /** Creates a new SpinIntake. */
  private  Intake intake;
  private  Supplier<PieceType> piece;

  public IntakePiece(Intake intake, Supplier<PieceType> piece) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.piece = piece;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.setIntakeSpeed(piece.get(), true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0.0);
    intake.setPIDController(intake.getEncoderPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isHoldingGamePiece();
  }
}
