// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class HoldArm extends CommandBase {
  private DoubleSupplier angleToHoldSupplier;
  private double angleToHold;
  private Arm arm;
  private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV);
  /** Creates a new HoldWrist. */
  public HoldArm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleToHold = -arm.getShoulderAngle();
    System.out.println("angle to hold is " + angleToHold);
  


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = feedforward.calculate(angleToHold, 0);
    System.out.println("Output is " + output);
    arm.setShoulderSpeed(output);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setShoulderSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
