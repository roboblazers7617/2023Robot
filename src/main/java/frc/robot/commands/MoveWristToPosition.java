// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.WristPosition;

public class MoveWristToPosition extends CommandBase {
  
  private WristPosition targetPosition;
  private Intake intake;
  private final ProfiledPIDController wristPID = new ProfiledPIDController(
            1, 0, 0, new TrapezoidProfile.Constraints(1.0, 0.25));

  /** Creates a new MoveWristToPosition. */
  public MoveWristToPosition(WristPosition targetPosition, Intake intake) {
  
    this.intake = intake;
    this.targetPosition = targetPosition;
    wristPID.setTolerance(1.0);

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    wristPID.setGoal(targetPosition.angle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setWristSpeed(wristPID.calculate(intake.getWristAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setWristSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristPID.atGoal();
  }
}
