// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ArmConstants.StateConstants.GenericPosition;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.States.StopIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stow extends ParallelCommandGroup {
  /** Creates a new Stow. */
  public Stow(Arm arm, Intake intake, boolean WaitUntilAtState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new StopIntake(intake), arm.changeState(() -> PieceType.Cone, GenericPosition.Stow, WaitUntilAtState));
  }
}
