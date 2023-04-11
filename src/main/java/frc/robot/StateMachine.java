// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.Constants.StateConstants.GenericPosition;
import frc.robot.Constants.StateConstants.State;
import frc.robot.Constants.WristConstants.WristPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class StateMachine extends SubsystemBase {
    private State currentState = State.Stow;
    private State targetState = State.Stow;
    private Arm arm;
    private Wrist wrist;
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

    public StateMachine(Arm arm, Wrist wrist){
        this.arm = arm;
        this.wrist = wrist;
    }
   
    @Override
    public void periodic(){
        if ((currentState != targetState) && arm.atSetpoint() 
        && (arm.getSuperstructureState() == currentState.getArmPosition().getPistonPosition().getValue()) && wrist.atSetpoint()){
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
            wrist.setPosition(currentState.getWristPosition(), () -> arm.getArmAngle());
            arm.setPosition(currentState.getArmPosition());
            arm.actuateSuperstructure(currentState.getArmPosition().getPistonPosition());
        }
    }

    public void changeState(PieceType piece, GenericPosition genericPosition){
       targetState = evalPosition(piece, genericPosition);
    }

    public Command changeStateCommand(Supplier<PieceType> piece, Supplier<GenericPosition> genericPosition){
        return new InstantCommand(() -> changeState(piece.get(), genericPosition.get()));
    }

    public Command changeStateCommand(Supplier<PieceType> piece, GenericPosition genericPosition){
        return new InstantCommand(() -> changeState(piece.get(), genericPosition));
    }
    public State getState(){
        return currentState;
    }

    public ArmPosition getStateArmPosition(){
        return currentState.getArmPosition();
    }

    public WristPosition getStateWristPosition(){
        return currentState.getWristPosition();
    }

    public boolean isAtState(){
        return ((currentState == targetState) && arm.atSetpoint()
         && (arm.getSuperstructureState() == currentState.getArmPosition().getPistonPosition().getValue()) && wrist.atSetpoint());
    }

    public Command waitUntilInPosition(){
        return Commands.waitUntil(() -> isAtState());
    }

    private State evalPosition(PieceType pieceType, GenericPosition genericPosition){
        for(int j = 1; j<stateDiagramList.length; j++){
            if(((stateDiagramList[j][0].getPiece() == pieceType) || (stateDiagramList[j][0].getPiece() == PieceType.NULL))
             && stateDiagramList[j][0].getPiece() == pieceType)
                     return stateDiagramList[j][0];
        }
        return currentState;
    }

    


}
