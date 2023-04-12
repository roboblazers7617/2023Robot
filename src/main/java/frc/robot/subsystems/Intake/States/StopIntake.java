// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.States;

import frc.robot.subsystems.Intake.Intake;
import frc.team4272.globals.State;

/** Add your docs here. */
public class StopIntake extends State<Intake> {
    public StopIntake(Intake intake){
        super(intake);
    }

    @Override
    public void initialize(){
        requiredSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
