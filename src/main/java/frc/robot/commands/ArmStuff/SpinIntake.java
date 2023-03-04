// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.WristConstants.IntakeConstants.IntakeDirection;
import frc.robot.subsystems.Intake;


public class SpinIntake extends CommandBase {
  /** Creates a new SpinIntake. */
  private  Intake intake;
  private  boolean isIntaking;
  private  PieceType piece;

  public SpinIntake(Intake intake, PieceType piece, Boolean isIntaking) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.isIntaking = isIntaking;
    this.piece = piece;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.setIntakeSpeed(piece, isIntaking);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return intake.isHoldingGamePiece();
    return false;
  }
}
