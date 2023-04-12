// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain.States;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.team4272.globals.State;

public class ManualControlState extends State<Drivetrain> {
  /** Creates a new ManualControlState. */
  Supplier<Double> leftY;
  Supplier<Double> rightY;
  Supplier<Double> rightX;
  Supplier<Boolean> isQuickTurn;
  public ManualControlState(Drivetrain drivetrain, Supplier<Double> leftY, Supplier<Double> rightX, Supplier<Double> rightY, Supplier<Boolean> isQuickTurn) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drivetrain);
    this.leftY = leftY;
    this.rightY = rightY;
    this.rightX = rightX;
    this.isQuickTurn = isQuickTurn;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    requiredSubsystem.drive(leftY, rightX, rightY, isQuickTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    requiredSubsystem.driveWithVelocity(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
