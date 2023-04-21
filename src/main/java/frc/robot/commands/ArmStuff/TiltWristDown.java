// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class TiltWristDown extends SequentialCommandGroup {
  /** Creates a new TiltWristDown. */
  public TiltWristDown(Arm arm, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new InstantCommand(() -> wrist.setPosition(wrist.getWristPosition()-18, () -> arm.getArmAngle())), 
    wrist.WaitUntilWristInPosition());
  }

}
