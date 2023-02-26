// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class HoldWrist extends CommandBase {
  private DoubleSupplier angleToHoldSupplier;
  private double angleToHold;
  private Intake intake;
  private ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.WRIST_KS, IntakeConstants.WRIST_KG, IntakeConstants.WRIST_KV);
  /** Creates a new HoldWrist. */
  public HoldWrist(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleToHold = intake.getEncoderAngle();
    System.out.println("angle to hold is " + angleToHold);
  


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = feedforward.calculate(angleToHold-20, 0);
    System.out.println("Output is " + output);
    intake.setWristSpeed(output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setWristSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
