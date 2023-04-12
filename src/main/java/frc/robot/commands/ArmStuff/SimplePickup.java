// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmStuff;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PieceType;
import frc.robot.Constants.ArmConstants.StateConstants.GenericPosition;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.States.IntakeForTime;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimplePickup extends SequentialCommandGroup {
  /** Creates a new SimplePickup. */
  public SimplePickup(Arm arm, Intake intake, Supplier<PieceType> piece,
      Supplier<GenericPosition> position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(arm.changeState(piece, position, true),
       new IntakeForTime(intake, piece, .8), new Stow(arm, intake, false));
  }
}
