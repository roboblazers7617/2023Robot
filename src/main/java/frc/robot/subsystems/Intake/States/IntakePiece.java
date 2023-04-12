// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.States;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PieceType;
import frc.robot.subsystems.Intake.Intake;
import frc.team4272.globals.State;


public class IntakePiece extends State<Intake> {
  /** Creates a new SpinIntake. */
  private  Supplier<PieceType> piece;

  public IntakePiece(Intake intake, Supplier<PieceType> piece) {
    super(intake);
    this.piece = piece;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    requiredSubsystem.setIntakeSpeed(piece, true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
