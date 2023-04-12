// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.States;

import java.util.function.Supplier;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.team4272.globals.State;

public class SetVelocitiesState extends State<Arm> {
  /** Creates a new SetVelocitiesState. */
  private Supplier<Double> shoulderVelocity;
  private Supplier<Double> wristVelocity;
  public SetVelocitiesState(Arm arm, Supplier<Double> shoulderVelocity, Supplier<Double> wristVelocity) {
    super(arm);
    this.shoulderVelocity = shoulderVelocity;
    this.wristVelocity = wristVelocity;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shoulderVelocity.get() > OperatorConstants.DEADZONE)
      requiredSubsystem.setShoulderVelocity(shoulderVelocity.get());
    else
      requiredSubsystem.setShoulderVelocity(0);
    
    if(wristVelocity.get() > OperatorConstants.DEADZONE)
      requiredSubsystem.setWristVelocity(wristVelocity.get());
    else
      requiredSubsystem.setWristVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
