// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ScoreGridSelection extends CommandBase {
  private int grid;
  private int position;
  private Drivetrain drivetrain;

  /** Creates a new GridSelection. */
  public ScoreGridSelection(Drivetrain drivetrain,int grid, int position) {
    this.grid = grid;
    this.position = position;
    this.drivetrain = drivetrain;
    
    drivetrain.setPathPlanningTarget(getPathPlanningTarget());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private String getPathPlanningTarget(){
    return "" + grid + position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("The grid is " + grid + ". The position is " + position);
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
