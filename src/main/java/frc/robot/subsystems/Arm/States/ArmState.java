// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.States;

import frc.robot.Constants.ArmConstants.StateConstants.StatePosition;
import frc.robot.subsystems.Arm.Arm;
import frc.team4272.globals.State;

/** Add your docs here. */
public class ArmState extends State<Arm> {
    private StatePosition state;

    public ArmState(Arm arm, StatePosition state){
        super(arm);
        this.state = state;
    }
    @Override
    public void initialize(){
        requiredSubsystem.setPosition(state);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
