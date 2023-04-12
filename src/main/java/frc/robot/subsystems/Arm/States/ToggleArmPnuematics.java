// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.ArmConstants.PnuematicsConstants.PnuematicPositions;
import frc.robot.subsystems.Arm.Arm;
import frc.team4272.globals.State;

public class ToggleArmPnuematics extends State<Arm> {
  /** Creates a new ToggleArmPnuematics. */
  Arm mArm;
  public ToggleArmPnuematics(Arm arm) {
    super (arm);
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
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
