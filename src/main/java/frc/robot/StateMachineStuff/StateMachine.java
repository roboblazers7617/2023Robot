// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.StateMachineStuff;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StateConstants.State;

/** Add your docs here. */
public class StateMachine extends SubsystemBase {
    private State currentState = State.Stow;
    private State targetState = State.Stow;
    private State[][] stateDiagramList = {
    {null,                   State.Stow,           State.ConeFloorPickup, State.CubeFloorPickup, State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2, State.CubeLevel2, State.ConeLevel3,     State.CubeLevel3},

    {State.Stow,             State.Stow,           State.LowTransition,   State.LowTransition,   State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2,     State.CubeLevel2,     State.HighTransition, State.HighTransition},
    {State.LowTransition,    State.Stow,           State.ConeFloorPickup, State.CubeFloorPickup, State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2,     State.CubeLevel2,     State.HighTransition, State.HighTransition},
    {State.HighTransition,   State.Stow,           State.ConeFloorPickup, State.CubeFloorPickup, State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2,     State.CubeLevel2,     State.ConeLevel3,     State.CubeLevel3},
    {State.ConeFloorPickup,  State.LowTransition,  State.ConeFloorPickup, State.CubeFloorPickup, State.ConeDoublePickup, State.CubeDoublePickup, State.LowTransition,  State.LowTransition,  State.ConeLevel2,     State.CubeLevel2,     State.HighTransition, State.HighTransition},
    {State.CubeFloorPickup,  State.LowTransition,  State.ConeFloorPickup, State.CubeFloorPickup, State.ConeDoublePickup, State.CubeDoublePickup, State.LowTransition,  State.LowTransition,  State.ConeLevel2,     State.CubeLevel2,     State.HighTransition, State.HighTransition},
    {State.ConeDoublePickup, State.Stow,           State.ConeFloorPickup, State.CubeFloorPickup, State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2,     State.CubeLevel2,     State.ConeLevel3,     State.CubeLevel3},
    {State.CubeDoublePickup, State.Stow,           State.ConeFloorPickup, State.CubeFloorPickup, State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2,     State.CubeLevel2,     State.ConeLevel3,     State.CubeLevel3},
    {State.ConeLevel1,       State.Stow,           State.LowTransition,   State.LowTransition,   State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2,     State.CubeLevel2,     State.HighTransition, State.HighTransition},
    {State.CubeLevel1,       State.Stow,           State.LowTransition,   State.LowTransition,   State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2,     State.CubeLevel2,     State.HighTransition, State.HighTransition},
    {State.ConeLevel2,       State.Stow,           State.LowTransition,   State.LowTransition,   State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2,     State.CubeLevel2,     State.ConeLevel3,     State.CubeLevel3},
    {State.CubeLevel2,       State.Stow,           State.LowTransition,   State.LowTransition,   State.ConeDoublePickup, State.CubeDoublePickup, State.ConeLevel1,     State.CubeLevel1,     State.ConeLevel2,     State.CubeLevel2,     State.HighTransition, State.HighTransition},
    {State.ConeLevel3,       State.HighTransition, State.HighTransition, State.HighTransition,   State.ConeDoublePickup, State.HighTransition,   State.HighTransition, State.HighTransition, State.HighTransition, State.HighTransition, State.ConeLevel3,     State.CubeLevel3},
    {State.CubeLevel3,       State.HighTransition, State.HighTransition, State.HighTransition,   State.ConeDoublePickup, State.HighTransition,   State.HighTransition, State.HighTransition, State.HighTransition, State.HighTransition, State.ConeLevel3,     State.CubeLevel3}};

    @Override
    public void periodic(){
        if (currentState != targetState){
            int targetIndex = 0;
            int currentIndex = 0;
            for(int i = 1; i<stateDiagramList[0].length; i++){
                if(stateDiagramList[0][i] == targetState)
                    targetIndex = i;
            }
            for(int j = 1; j<stateDiagramList.length; j++){
                    if(stateDiagramList[j][0] == currentState)
                            currentIndex = j;
            }
            if((targetIndex != 0) && (currentIndex != 0))
                currentState = stateDiagramList[currentIndex][targetIndex];
        }
    }

    public void changeState(State desiredState){
       targetState = desiredState;
    }
    public State getState(){
        return currentState;
    }


}
