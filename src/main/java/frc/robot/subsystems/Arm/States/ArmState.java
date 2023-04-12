// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import frc.robot.Constants.ArmConstants.StateConstants;
import frc.robot.Constants.ArmConstants.ShoulderConstants.ArmPosition;
import frc.robot.Constants.ArmConstants.WristConstants.WristPosition;
import frc.team4272.globals.State;
import frc.robot.Constants.ArmConstants.StateConstants;

/** Add your docs here. */
public class ArmState extends State<Arm> {
    private StateConstants.State positionState;

    public ArmState(Arm arm, StateConstants.State state){
        super(arm);
        positionState = state;
    }
    @Override
    public void initialize(){
        requiredSubsystem.setPosition(positionState.getArmPosition(), positionState.getWristPosition());
    }

}
