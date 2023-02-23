// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PnuematicsConstants.PnuematicPositions;
import frc.robot.subsystems.Arm;

public class ToggleArmPnuematics extends CommandBase {
  /** Creates a new ToggleArmPnuematics. */
  Arm mArm;
  public ToggleArmPnuematics(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    mArm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mArm.getSuperstructureState() == Value.kForward){
      mArm.actuateSuperstructure(PnuematicPositions.RETRACTED);
    }

    else if(mArm.getSuperstructureState() == Value.kReverse || mArm.getSuperstructureState() == Value.kOff){
      mArm.actuateSuperstructure(PnuematicPositions.EXTENDED);
    }
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
