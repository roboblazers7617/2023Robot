// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.States;

import frc.robot.subsystems.Arm.Arm;
import frc.team4272.globals.State;

public class WaitUntilAtState extends State<Arm> {
  /** Creates a new WaitTillAtTargetState. */
  public WaitUntilAtState(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(arm);
  }
  
  @Override
  public boolean isFinished() {
    return (requiredSubsystem.atSetpoints() && (requiredSubsystem.getCurrentState() == requiredSubsystem.getTargetState()));
  }
}
